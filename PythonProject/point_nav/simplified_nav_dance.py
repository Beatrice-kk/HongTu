#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import threading
import time
import tf.transformations as tft
import argparse
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback
from actionlib_msgs.msg import GoalID, GoalStatusArray
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool

class SimpleNavWaypointPlayer:
    def __init__(self, backstage_pos, door_pos, dance_type, dance_choreography, threshold,threshold_yaw):
        """
        Initialize the navigator with waypoint information.
        """
        # Check if dance type exists in choreography
        if dance_type not in dance_choreography:
            rospy.logerr(f"Dance type '{dance_type}' not defined, using default choreography")
            self.dance_sequence = list(dance_choreography.values())[0]
        else:
            self.dance_sequence = dance_choreography[dance_type]
        
        # Extract dance positions and wait times
        dance_waypoints = [pos for pos, _ in self.dance_sequence]
        self.wait_times = [wait for _, wait in self.dance_sequence]
        
        # Build complete waypoint sequence
        self.waypoints = dance_waypoints  +[backstage_pos]
        rospy.loginfo(f"总路径点数量: {len(self.waypoints)}，舞蹈点: {len(dance_waypoints)}，最后还会回到: {backstage_pos}")
      
        self.threshold = threshold
        self.threshold_yaw=threshold_yaw
        self.current_waypoint_index = 0
        self.reached_final = False
        
        # Current position tracking
        self.current_position = {"x": 0.0, "y": 0.0, "theta": 0.0}
        
        # Dance state tracking
        self.dance_service_called = False
        self.dance_type = dance_type
        self.dance_in_progress = False
        self.navigation_active = False
        
        # Navigation status monitoring
        self.move_base_status = None
        self.last_goal_send_time = None
        self.goal_send_retries = 0
        self.max_goal_retries = 3
        self.skip_count = 0  # 记录跳过的点数
        self.max_skip_count = 3  # 最多允许跳过的点数
        
        # 添加停止状态跟踪
        self.stop_enforcer_timer = None
        self.completely_stopped = False
        
        # Publishers and subscribers
        self.goal_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=1)
        self.feedback_sub = rospy.Subscriber("/move_base/feedback", MoveBaseActionFeedback, self.feedback_callback)
        self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
      #   self.odom_sub = rospy.Subscriber("/slam_odom", Odometry, self.odom_callback)
        self.dance_direction_pub = rospy.Publisher('dance_direction', String, queue_size=1)
        self.status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_callback)
        
        # Add cmd_vel publisher to stop the robot
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Dance service client
        rospy.loginfo("Waiting for dance service...")
        try:
            rospy.wait_for_service('play_dance', timeout=5.0)
            self.play_dance_service = rospy.ServiceProxy('play_dance', Trigger)
            rospy.loginfo("Dance service connected")
        except rospy.ROSException:
            rospy.logwarn("Dance service not available, will not perform dance actions")
            self.play_dance_service = None
        
        # Navigation watchdog timer
        self.nav_watchdog_timer = None
        self.last_position_check = {"x": 0.0, "y": 0.0, "time": rospy.Time.now()}
        
        # Add a timer for dance completion
        self.dance_timer = None
        
        rospy.sleep(1.0)
        rospy.loginfo(f"Starting performance, dance type: {self.dance_type}")
        self.start_navigation_watchdog()
        self.navigate_to_current_waypoint()

    def status_callback(self, msg):
        """Monitor move_base status"""
        self.move_base_status = msg
    
    def start_navigation_watchdog(self):
        """Start a timer to periodically check if navigation is progressing"""
        if self.nav_watchdog_timer:
            self.nav_watchdog_timer.shutdown()
        self.nav_watchdog_timer = rospy.Timer(rospy.Duration(5.0), self.check_navigation_progress)
      
      
    def segment_nav_to_backstage(self, target, step_size=0.5, threshold=0.5, max_steps=20):
      """分段导航到后台点：自动生成中间点，逐步导航"""
      current_x = self.current_position["x"]
      current_y = self.current_position["y"]
      target_x, target_y, target_theta = target

      path = []
      dx = target_x - current_x
      dy = target_y - current_y
      dist = math.hypot(dx, dy)
      steps = int(dist // step_size) + 1

      for i in range(1, min(steps+1, max_steps+1)):
         ratio = i / steps
         intermediate_x = current_x + dx * ratio
         intermediate_y = current_y + dy * ratio
         path.append((intermediate_x, intermediate_y, target_theta))

      path.append((target_x, target_y, target_theta))  # 确保最后一个点是后台点

      rospy.loginfo(f"分段导航：生成{len(path)}个中间点")
      for pt in path:
         # 尝试导航到每个点
         goal_msg = self._build_move_base_goal(*pt)
         self.goal_pub.publish(goal_msg)
         self.last_goal_send_time = rospy.Time.now()
         self.navigation_active = True

         # 等待到达或timeout
         start_time = time.time()
         while True:
               dx = self.current_position["x"] - pt[0]
               dy = self.current_position["y"] - pt[1]
               dist = math.hypot(dx, dy)
               if (dist < threshold):
                  rospy.loginfo(f"到达分段点: ({pt[0]:.2f}, {pt[1]:.2f})")
                  break
               if time.time() - start_time > 15.0:  # 每个点最多等待15s
                  rospy.logwarn("分段点导航超时或失败，尝试下一个点")
                  break
               rospy.sleep(0.2)

      rospy.loginfo("后台点分段导航结束")
    def check_navigation_progress(self, event):
        """Check if the robot is making progress toward its goal"""
        if not self.navigation_active or self.dance_in_progress or self.reached_final:
            return
            
        # Check if we've moved since last check
        current_time = rospy.Time.now()
        dx = self.current_position["x"] - self.last_position_check["x"]
        dy = self.current_position["y"] - self.last_position_check["y"]
        dist_moved = math.hypot(dx, dy)
        time_diff = (current_time - self.last_position_check["time"]).to_sec()
        
        # Update last check
        self.last_position_check["x"] = self.current_position["x"]
        self.last_position_check["y"] = self.current_position["y"]
        self.last_position_check["time"] = current_time
        
        # If we've barely moved in 5 seconds, we might be stuck
        if dist_moved < 0.05 and time_diff > 4.0 and self.navigation_active:
            rospy.logwarn(f"[卡住了] 机器人在{time_diff:.1f}秒内只移动了{dist_moved:.3f}米，尝试恢复...")
            
            # Check if we've sent the goal recently
            if self.last_goal_send_time and (current_time - self.last_goal_send_time).to_sec() < 10.0:
                self.goal_send_retries += 1
                
                if self.goal_send_retries >= self.max_goal_retries:
                   
                   if self.current_waypoint_index == len(self.waypoints) - 1:
                        rospy.logwarn("后台点规划失败，启动分段导航")
                        self.segment_nav_to_backstage(self.waypoints[self.current_waypoint_index])
                        self.reached_final = True
                        return
     
                   else:
                    self.skip_count += 1
                    rospy.logwarn(f"[导航失败] 多次尝试设置目标失败。正在强制移动到下一个路径点。已跳过{self.skip_count}个点。")
                    
                    # 记录当前路径点索引，用于调试
                    current_idx = self.current_waypoint_index
                    self.current_waypoint_index += 1
                    self.goal_send_retries = 0
                    
                    # 检查是否还有剩余路径点
                    if self.current_waypoint_index < len(self.waypoints):
                        rospy.loginfo(f"[继续导航] 从路径点{current_idx+1}移动到路径点{self.current_waypoint_index+1}，总共{len(self.waypoints)}个点")
                        self.navigate_to_current_waypoint()
                    else:
                        if self.skip_count >= self.max_skip_count:
                            rospy.logwarn(f"[完成] 已跳过{self.skip_count}个点，超过最大允许值。结束导航。")
                            self.reached_final = True
                        else:
                            rospy.loginfo("[完成] 所有路径点已完成!")
                            self.reached_final = True
                    return
            
            # Try to re-send the goal after forcefully clearing move_base
            rospy.loginfo("重置导航系统并重新发送目标...")
            self.reset_navigation()
            
            # Re-send current goal
            if self.current_waypoint_index < len(self.waypoints):
                self.navigate_to_current_waypoint(is_retry=True)

    def reset_navigation(self):
        """Reset the navigation system to recover from stuck state"""
        # First cancel any existing goals
        cancel_msg = GoalID()
        for _ in range(3):  # Send multiple times to ensure it's received
            self.cancel_pub.publish(cancel_msg)
            rospy.sleep(0.1)
        
        # Stop the robot
        self.stop_robot()
        
        # Wait for move_base to clear
        rospy.sleep(1.0)

    def start_stop_enforcer(self):
        """Start a timer to continuously send stop commands to prevent adjustment movements"""
        if self.stop_enforcer_timer:
            self.stop_enforcer_timer.shutdown()
        
        self.completely_stopped = False
        # 每0.5秒发送一次停止命令，确保机器人不会进行微调
        self.stop_enforcer_timer = rospy.Timer(rospy.Duration(0.5), self.enforce_stop)
        rospy.loginfo("启动停止强制器 - 防止机器人踏步调整")
    
    def stop_stop_enforcer(self):
        """Stop the enforcer timer when we're ready to move again"""
        if self.stop_enforcer_timer:
            self.stop_enforcer_timer.shutdown()
            self.stop_enforcer_timer = None
            rospy.loginfo("停止强制器已关闭 - 机器人可以恢复移动")
    
    def enforce_stop(self, event):
        """Timer callback to continuously force the robot to stay still"""
        if self.dance_in_progress and not self.completely_stopped:
            self.stop_robot()
            
            # 取消所有导航目标，确保move_base不会继续调整位置
            cancel_msg = GoalID()
            self.cancel_pub.publish(cancel_msg)

    def stop_robot(self):
        """Send zero velocity command to stop the robot completely"""
        zero_vel = Twist()
        zero_vel.linear.x = 0.0
        zero_vel.linear.y = 0.0
        zero_vel.linear.z = 0.0
        zero_vel.angular.x = 0.0
        zero_vel.angular.y = 0.0
        zero_vel.angular.z = 0.0
        
        # Publish multiple times to ensure the command is received
        for _ in range(3):
            self.cmd_vel_pub.publish(zero_vel)
            rospy.sleep(0.05)
        
        rospy.loginfo("机器人完全停止")
        self.completely_stopped = True

    def _build_move_base_goal(self, x, y, theta_deg):
        """
        Construct a MoveBaseActionGoal message for publishing to move_base/goal
        """
        goal = MoveBaseActionGoal()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.goal_id.stamp = rospy.Time.now()
        # Generate a unique ID for each goal
        goal.goal_id.id = f"nav_dance_{rospy.Time.now().to_nsec()}"
        goal.goal.target_pose.header.frame_id = "map"
        goal.goal.target_pose.header.stamp = rospy.Time.now()
        goal.goal.target_pose.pose.position.x = x
        goal.goal.target_pose.pose.position.y = y
        q = tft.quaternion_from_euler(0, 0, math.radians(theta_deg))
        goal.goal.target_pose.pose.orientation.x = q[0]
        goal.goal.target_pose.pose.orientation.y = q[1]
        goal.goal.target_pose.pose.orientation.z = q[2]
        goal.goal.target_pose.pose.orientation.w = q[3]
        return goal

   #  def odom_callback(self, msg):
   #      """Get current position from odometry as backup"""
   #      self.current_position["x"] = msg.pose.pose.position.x
   #      self.current_position["y"] = msg.pose.pose.position.y
        
   #      # Extract angle
   #      orientation = msg.pose.pose.orientation
   #      quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
   #      euler = tft.euler_from_quaternion(quaternion)
   #      self.current_position["theta"] = math.degrees(euler[2])

    def navigate_to_current_waypoint(self, is_retry=False):
        """Navigate to the current waypoint"""
        if self.current_waypoint_index >= len(self.waypoints):
            rospy.loginfo("[完成] 所有路径点已完成!")
            self.reached_final = True
            return

        # 确保停止强制器已关闭
        self.stop_stop_enforcer()
        self.completely_stopped = False

        # Reset for new navigation attempt
        if not is_retry:
            self.goal_send_retries = 0
            
        # First ensure any previous goals are canceled
        self.reset_navigation()

        # Get current waypoint
        x, y, theta = self.waypoints[self.current_waypoint_index]
        
        # Determine location description
        if self.current_waypoint_index == len(self.waypoints) - 1:
            location_desc = "[返回后台]"
            #启用旋转
            set_rotation(True)
        elif self.current_waypoint_index==1:
            location_desc = f"[舞蹈位置 {self.current_waypoint_index+1}]"
            print("到达舞台第一个点   禁用旋转")
            set_rotation(False)

            
        else:
            location_desc = f"[舞蹈位置 {self.current_waypoint_index+1}]"

        wait_time = self.wait_times[self.current_waypoint_index] if self.current_waypoint_index < len(self.wait_times) else 0
        prefix = "[重试] " if is_retry else ""
        rospy.loginfo(f"{prefix}[导航目标 {self.current_waypoint_index+1}/{len(self.waypoints)}] {location_desc} x={x}, y={y}, θ={theta} (等待时间: {wait_time}秒)")
        
        # Update last position check
        self.last_position_check["x"] = self.current_position["x"]
        self.last_position_check["y"] = self.current_position["y"]
        self.last_position_check["time"] = rospy.Time.now()
        
        # Publish MoveBaseActionGoal
        goal_msg = self._build_move_base_goal(x, y, theta)
        self.goal_pub.publish(goal_msg)
        self.last_goal_send_time = rospy.Time.now()
        self.navigation_active = True
        
        # Make sure cmd_vel is not zero after setting a new goal
        # This helps in case the robot is "stuck" in a stopped state
        rospy.sleep(0.5)
        self.cmd_vel_pub.publish(Twist())  # Empty twist = not forcing zero

    def perform_dance(self):
        """
        Call the dance service, only execute on first call
        """
        if self.dance_service_called:
            rospy.loginfo("舞蹈服务已被调用，不再重复调用")
            return
            
        if self.play_dance_service is None:
            rospy.logwarn("舞蹈服务不可用，跳过舞蹈")
            return
            
        try:
            dance_direction = self.dance_type
            rospy.loginfo(f"开始舞蹈: {dance_direction}，在此表演中只会调用一次舞蹈服务")
            self.dance_direction_pub.publish(String(dance_direction))
            rospy.sleep(0.5)
            
            # 在真正的非阻塞方式中调用服务
            dance_thread = threading.Thread(target=self._call_dance_service)
            dance_thread.daemon = True
            dance_thread.start()
            
            # 标记舞蹈服务已调用
            self.dance_service_called = True
            rospy.loginfo("舞蹈服务调用已启动，等待完成后将继续导航")
            
            # 设置一个定时器，确保即使舞蹈服务卡住也能继续
            self.schedule_next_waypoint()
            
        except Exception as e:
            rospy.logerr(f"舞蹈服务调用失败: {e}")
            # 即使舞蹈失败，也应该继续

    def _call_dance_service(self):
        """Helper method to call dance service in a separate thread"""
        try:
            self.play_dance_service()
            rospy.loginfo("舞蹈服务已完成")
        except Exception as e:
            rospy.logerr(f"舞蹈服务执行错误: {e}")

    def schedule_next_waypoint(self):
        """Schedule movement to next waypoint after waiting period"""
        # Get wait time for current position
        wait_time = self.wait_times[self.current_waypoint_index] if self.current_waypoint_index < len(self.wait_times) else 0
        
        # If we have an existing timer, cancel it
        if self.dance_timer:
            self.dance_timer.shutdown()
            
        # Schedule the continuation after the wait time
        rospy.loginfo(f"计划在{wait_time}秒后移动到下一个路径点")
        self.dance_timer = rospy.Timer(rospy.Duration(wait_time), self.continue_to_next_waypoint, oneshot=True)

    def continue_to_next_waypoint(self, event=None):
        """Timer callback to continue to next waypoint"""
        rospy.loginfo("等待时间结束，继续前往下一个路径点")
        
        # 确保停止强制器已关闭，以便机器人可以开始移动
        self.stop_stop_enforcer()
        
        self.dance_in_progress = False
        self.current_waypoint_index += 1
        
        if self.current_waypoint_index < len(self.waypoints):
            rospy.loginfo(f"移动到路径点 {self.current_waypoint_index+1}/{len(self.waypoints)}")
            self.navigate_to_current_waypoint()
        else:
            rospy.loginfo("[完成] 所有路径点已完成!")
            self.reached_final = True

    def execute_dance_and_wait(self):
        """Execute dance, wait for specified time, then continue"""
        try:
            # 开始强制停止，防止踏步调整
            self.start_stop_enforcer()
            
            # 强制停止机器人
            self.stop_robot()
            
            # 确保取消所有导航目标
            cancel_msg = GoalID()
            self.cancel_pub.publish(cancel_msg)
            
            # Check if we should start dancing (at first dance position)
            if self.current_waypoint_index == 0 and not self.dance_service_called:
                rospy.loginfo("/* 已到达第一个舞蹈位置，开始舞蹈... */")
                self.perform_dance()
                # The scheduling is now handled by perform_dance(), so we can return
                return
            
            # For other positions, just wait and then continue
            wait_time = self.wait_times[self.current_waypoint_index] if self.current_waypoint_index < len(self.wait_times) else 0
            
            if wait_time > 0:
                rospy.loginfo(f"等待{wait_time}秒后再前往下一个路径点...")
                # Schedule the next waypoint timer
                self.schedule_next_waypoint()
            else:
                # Continue immediately to next waypoint
                rospy.loginfo("立即前往下一个路径点...")
                self.continue_to_next_waypoint()
                
        except Exception as e:
            rospy.logerr(f"舞蹈执行错误: {e}")
            # Ensure we continue even if there's an error
            self.stop_stop_enforcer()
            self.dance_in_progress = False
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                self.navigate_to_current_waypoint()

    def feedback_callback(self, msg):
        """Handle navigation feedback, detect arrival"""
        if self.reached_final or self.dance_in_progress:
            return

        # Update current position
        
        current_pose = msg.feedback.base_position.pose
        self.current_position["x"] = current_pose.position.x
        self.current_position["y"] = current_pose.position.y
        
        orientation = current_pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = tft.euler_from_quaternion(quaternion)
        self.current_position["theta"] = math.degrees(euler[2])

      # Get current waypoint
        x, y, target_yaw = self.waypoints[self.current_waypoint_index]

      # Calculate distance to target
        dx = current_pose.position.x - x
        dy = current_pose.position.y - y
        dist = math.hypot(dx, dy)

      # Calculate angular difference
        current_yaw = self.current_position["theta"]  # Assuming this is in degrees
        d_yaw = abs(current_yaw - target_yaw)
       # Normalize to the range [0, 180]
        if d_yaw > 180:
           d_yaw = 360 - d_yaw

      # Log information
        rospy.loginfo_throttle(2, f"[当前位置] ({current_pose.position.x:.2f}, {current_pose.position.y:.2f}) -> 距离目标: {dist:.2f} 米, 角度差: {d_yaw:.2f} 度")

      # Check if we've reached the waypoint
        if dist <= self.threshold and d_yaw <= self.threshold_yaw and not self.dance_in_progress:
      #   if dist <= self.threshold and not self.dance_in_progress and :
            rospy.loginfo(f"到达路径点 {self.current_waypoint_index+1}")
            self.navigation_active = False
            self.dance_in_progress = True
            
            # 立即停止机器人，防止微调
            self.stop_robot()
            cancel_msg = GoalID()
            self.cancel_pub.publish(cancel_msg)
            
            # Execute dance and wait in a separate thread
            dance_thread = threading.Thread(target=self.execute_dance_and_wait)
            dance_thread.daemon = True
            dance_thread.start()
def set_rotation(enable: bool):
    rospy.wait_for_service('/unitree_cmd_vel_controller/set_rotation_enabled')
    try:
        set_rotation_srv = rospy.ServiceProxy('/unitree_cmd_vel_controller/set_rotation_enabled', SetBool)
        resp = set_rotation_srv(enable)
        print(f"Service response: success={resp.success}, message={resp.message}")
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

if __name__ == "__main__":
   
    set_rotation(True)   #先启用 第一个点后禁用旋转
    rospy.init_node("simple_nav_waypoints_player")
    parser = argparse.ArgumentParser(description='Navigation Dance Performance Controller')
    parser.add_argument('--dance', type=str, default='A', 
                      choices=['A', 'B', 'Up', 'Down', 'Left', 'Right', 'X', 'Y'],
                      help='Specify dance type to execute')
    args, unknown = parser.parse_known_args()
    
    backstage_pos = (-0.6, 0, 0)
    stage_entry_pos = (-1.94, 0.80, 115)  #弃用
    door_pos = (-1.6, 0, 0)

    dance_choreography = {
        'A': [
            ((-3.2, 3.8, -160), 20.0),
            ((-3.4, 3.2, 180), 30.0),
            ((-3.6, 2.5, 150), 20.0),
        ],
        'B': [
            ((4.18, 1.15, -159), 2.0),
            ((-3.5, 0.5, 90), 4.0),
            ((2.3, 1.7, -45), 3.0),
            ((-1.5, -1.2, 180), 2.5),
        ],
        'Up': [
            ((4.18, 1.15, -159), 2.5),
            ((3.0, 3.0, 0), 3.0),
            ((-2.0, 2.0, 45), 4.0),
        ],
        'Down': [
            ((4.18, 1.15, -159), 1.5),
            ((0.0, 0.0, -90), 3.5),
            ((2.5, -2.0, 135), 2.0),
            ((-2.5, -1.0, -135), 4.5),
        ],
        'Left': [
            ((4.18, 1.15, -159), 2.0),
            ((-4.0, 1.0, 90), 3.0),
            ((-2.0, -1.0, -90), 2.5),
        ],
        'Right': [
            ((4.18, 1.15, -159), 2.0),
            ((3.0, -1.0, -90), 3.0),
            ((1.0, 2.0, 0), 2.5),
        ],
        'X': [
            ((4.18, 1.15, -159), 3.0),
            ((-3.0, 3.0, 135), 4.0),
            ((3.0, -3.0, -45), 3.5),
            ((-3.0, -3.0, -135), 2.5),
            ((3.0, 3.0, 45), 3.0),
        ],
        'Y': [
            ((4.18, 1.15, -159), 2.5),
            ((0.0, 3.0, 90), 3.0),
            ((-2.0, 0.0, 180), 4.0),
            ((2.0, 0.0, 0), 3.5),
        ]
    }
    
    threshold_dist = 0.5  # Arrival threshold
    threshold_yaw = 20
    node = SimpleNavWaypointPlayer(
        backstage_pos=backstage_pos,
        door_pos=door_pos,
        dance_type=args.dance,
        dance_choreography=dance_choreography,
        threshold=threshold_dist,
        threshold_yaw=threshold_yaw
    )
    
    rospy.loginfo(f"表演开始，使用舞蹈类型: {args.dance}")
    rospy.spin()