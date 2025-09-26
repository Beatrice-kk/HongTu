#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import threading
import time
import tf.transformations as tft
import argparse
import sys
import os
from enum import Enum
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback
from actionlib_msgs.msg import GoalID, GoalStatusArray
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist

# 添加SDK路径以导入G1ActionPlayer
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../unitree_sdk2_python/example/g1/high_level/"))
from g1_action_player import G1ActionPlayer

class NavigationState(Enum):
    """导航状态枚举"""
    IDLE = "idle"
    NAVIGATING = "navigating"
    DANCING = "dancing"
    WAITING = "waiting"
    COMPLETED = "completed"
    ERROR = "error"


class SimpleNavWaypointPlayer:
    def __init__(
        self,
        backstage_pos,
        dance_type,
        dance_choreography,
    ):
        """
        Initialize the navigator with waypoint information.
        """
        # 线程安全锁
        self._lock = threading.RLock()
        
        # Check if dance type exists in choreography
        if dance_type not in dance_choreography:
            rospy.logerr(
                f"Dance type '{dance_type}' not defined, using default choreography"
            )
            self.dance_sequence = list(dance_choreography.values())[0]
        else:
            self.dance_sequence = dance_choreography[dance_type]

        # Extract dance positions and wait times
        dance_waypoints = [pos for pos, _ in self.dance_sequence]
        self.wait_times = [wait for _, wait in self.dance_sequence]

        # Build complete waypoint sequence
        # 确保后台点被包含在路径中
        if dance_waypoints and dance_waypoints[-1] != backstage_pos:
            self.waypoints = dance_waypoints + [backstage_pos]
        else:
            self.waypoints = dance_waypoints

        rospy.loginfo(
            f"总路径点数量: {len(self.waypoints)}，舞蹈点: {len(dance_waypoints)}，后台点: {backstage_pos}"
        )
        rospy.loginfo(f"完整路径点序列: {self.waypoints}")

        # 设置宽松的到达阈值，避免过度调整
        self.threshold = 0.5  # 距离阈值：0.5米内认为到达
        self.angle_threshold = 30.0  # 角度阈值：30度内认为到达

        # 使用状态机管理状态
        self._state = NavigationState.IDLE
        self.current_waypoint_index = 0
        self.reached_final = False

        # Current position tracking
        self.current_position = {"x": 0.0, "y": 0.0, "theta": 0.0}

        # Dance state tracking
        self.dance_service_called = False
        self.dance_type = dance_type

        # Navigation status monitoring
        self.move_base_status = None
        self.last_goal_send_time = None
        self.goal_send_retries = 0
        self.max_goal_retries = 2  # 减少重试次数，更快地前进到下一个点

        # 添加时间阈值 - 在一个点停留太久后自动前进
        self.waypoint_start_time = None
        
        # 定时器管理
        self._timers = []
        self._threads = []

        # Publishers and subscribers
        self.goal_pub = rospy.Publisher(
            "/move_base/goal", MoveBaseActionGoal, queue_size=1
        )
        self.feedback_sub = rospy.Subscriber(
            "/move_base/feedback", MoveBaseActionFeedback, self.feedback_callback
        )
        self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self.dance_direction_pub = rospy.Publisher(
            "dance_direction", String, queue_size=1
        )
        self.status_sub = rospy.Subscriber(
            "/move_base/status", GoalStatusArray, self.status_callback
        )

        # Add cmd_vel publisher to stop the robot
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        # Subscribe to cmd_vel to monitor robot movement
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        self.current_velocity = {"linear": 0.0, "angular": 0.0}
        self.last_velocity_check = rospy.Time.now()

        # Dance service client
        rospy.loginfo("Waiting for dance service...")
        try:
            rospy.wait_for_service("play_dance", timeout=5.0)
            self.play_dance_service = rospy.ServiceProxy("play_dance", Trigger)
            rospy.loginfo("Dance service connected")
        except rospy.ROSException:
            rospy.logwarn("Dance service not available, will not perform dance actions")
            self.play_dance_service = None

        # Navigation watchdog timer - reduced checking frequency
        self.nav_watchdog_timer = None
        self.last_position_check = {"x": 0.0, "y": 0.0, "time": rospy.Time.now()}

        # Add a timer for dance completion
        self.dance_timer = None

        # 添加一个持续移动的定时器，确保机器人不会停止
        self.movement_check_timer = rospy.Timer(
            rospy.Duration(3.0), self.ensure_movement
        )
        self._timers.append(self.movement_check_timer)
        
        # 添加速度检测相关变量
        self.stopped_timer = None
        self.stopped_duration = 0.0  # 停止持续时间
        self.required_stop_time = 0.5  # 需要停止0.5秒才认为完全停止（放宽条件）
        self.last_velocity_check_time = rospy.Time.now()
        
        # 直接位置环控制参数
        self.direct_control_enabled = False
        self.direct_control_timer = None
        self.position_kp = 0.5  # 位置比例增益
        self.angle_kp = 0.8    # 角度比例增益
        self.max_linear_vel = 0.3  # 最大线速度
        self.max_angular_vel = 0.5  # 最大角速度
        
        # 添加状态监控定时器
        self.status_timer = rospy.Timer(
            rospy.Duration(10.0), self.log_status_info
        )
        self._timers.append(self.status_timer)

        rospy.sleep(1.0)
        rospy.loginfo(f"Starting performance, dance type: {self.dance_type}")
        rospy.loginfo(f"总路径点数量: {len(self.waypoints)}")
        rospy.loginfo(f"第一个路径点: {self.waypoints[0] if self.waypoints else 'None'}")
        self.start_navigation_watchdog()
        rospy.loginfo("开始导航到第一个路径点...")
        self.navigate_to_current_waypoint()
        self.waypoint_start_time = rospy.Time.now()
        rospy.loginfo("导航启动完成")
        
        # 注册关闭处理程序
        rospy.on_shutdown(self.cleanup)
    
    def set_state(self, new_state):
        """线程安全的状态设置"""
        with self._lock:
            old_state = self._state
            self._state = new_state
            rospy.loginfo(f"状态变更: {old_state.value} -> {new_state.value}")
    
    def get_state(self):
        """线程安全的状态获取"""
        with self._lock:
            return self._state
    
    def cleanup(self):
        """清理资源"""
        rospy.loginfo("开始清理资源...")
        
        # 停止直接控制
        self.stop_direct_control()
        
        # 停止所有定时器
        for timer in self._timers:
            try:
                timer.shutdown()
            except Exception as e:
                rospy.logwarn(f"定时器关闭失败: {e}")
        
        # 等待所有线程结束
        for thread in self._threads:
            if thread.is_alive():
                thread.join(timeout=1.0)
        
        # 取消当前目标
        try:
            cancel_msg = GoalID()
            self.cancel_pub.publish(cancel_msg)
        except Exception as e:
            rospy.logwarn(f"取消目标失败: {e}")
        
        # 停止机器人
        try:
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
        except Exception as e:
            rospy.logwarn(f"停止机器人失败: {e}")
        
        rospy.loginfo("资源清理完成")
    
    def get_status_info(self):
        """获取当前状态信息，用于调试和监控"""
        with self._lock:
            return {
                "state": self._state.value,
                "current_waypoint": self.current_waypoint_index,
                "total_waypoints": len(self.waypoints),
                "position": self.current_position.copy(),
                "dance_service_called": self.dance_service_called,
                "reached_final": self.reached_final,
                "active_timers": len(self._timers),
                "active_threads": len([t for t in self._threads if t.is_alive()])
            }
    
    def log_status_info(self, event):
        """定期记录状态信息"""
        status = self.get_status_info()
        rospy.loginfo(f"[状态监控] {status}")

    def status_callback(self, msg):
        """Monitor move_base status"""
        self.move_base_status = msg
    
    def cmd_vel_callback(self, msg):
        """Monitor robot velocity to detect when it's stationary"""
        self.current_velocity["linear"] = math.sqrt(
            msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2
        )
        self.current_velocity["angular"] = math.sqrt(
            msg.angular.x**2 + msg.angular.y**2 + msg.angular.z**2
        )
        self.last_velocity_check = rospy.Time.now()
        
        # 检测是否完全停止
        current_time = rospy.Time.now()
        time_diff = (current_time - self.last_velocity_check_time).to_sec()
        
        # 如果速度很小，累积停止时间（放宽速度阈值）
        if (self.current_velocity["linear"] < 0.1 and 
            self.current_velocity["angular"] < 0.1):
            self.stopped_duration += time_diff
        else:
            # 如果还在移动，重置停止时间
            self.stopped_duration = 0.0
            
        self.last_velocity_check_time = current_time
    
    def start_direct_control(self, target_x, target_y, target_theta):
        """启动直接位置环控制"""
        rospy.loginfo(f"启动直接位置环控制到目标: ({target_x:.2f}, {target_y:.2f}, {target_theta:.1f}°)")
        self.direct_control_enabled = True
        
        # 创建直接控制定时器
        if self.direct_control_timer:
            self.direct_control_timer.shutdown()
        
        self.direct_control_timer = rospy.Timer(
            rospy.Duration(0.1),  # 10Hz控制频率
            lambda event: self.direct_position_control(target_x, target_y, target_theta)
        )
        self._timers.append(self.direct_control_timer)
    
    def stop_direct_control(self):
        """停止直接位置环控制"""
        if self.direct_control_timer:
            self.direct_control_timer.shutdown()
            self.direct_control_timer = None
        self.direct_control_enabled = False
        rospy.loginfo("停止直接位置环控制")
    
    def direct_position_control(self, target_x, target_y, target_theta):
        """直接位置环控制实现"""
        if not self.direct_control_enabled:
            return
            
        # 获取当前位置
        current_x = self.current_position["x"]
        current_y = self.current_position["y"]
        current_theta = self.current_position["theta"]
        
        # 计算位置误差
        dx = target_x - current_x
        dy = target_y - current_y
        dist_error = math.hypot(dx, dy)
        
        # 计算角度误差（最短路径）
        d_theta = target_theta - current_theta  # 修正：目标角度减去当前角度
        while d_theta > 180:
            d_theta -= 360
        while d_theta < -180:
            d_theta += 360
        
        # 检查是否到达目标
        if dist_error < self.threshold and abs(d_theta) < self.angle_threshold:
            rospy.loginfo(f"直接控制到达目标 (距离:{dist_error:.3f}m, 角度差:{d_theta:.1f}°)")
            self.stop_direct_control()
            # 停止机器人
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            
            # 处理到达目标后的逻辑
            self.handle_waypoint_arrival()
            return
        
        # 计算控制输出
        # 线速度控制
        linear_vel = self.position_kp * dist_error
        linear_vel = max(-self.max_linear_vel, min(linear_vel, self.max_linear_vel))
        
        # 角速度控制
        angular_vel = self.angle_kp * d_theta  # 移除负号，因为角度误差计算已修正
        angular_vel = max(-self.max_angular_vel, min(angular_vel, self.max_angular_vel))
        
        # 发布速度命令
        cmd_msg = Twist()
        cmd_msg.linear.x = linear_vel
        cmd_msg.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd_msg)
        
        # 定期打印控制信息
        rospy.loginfo_throttle(
            2,
            f"[直接控制] 目标:({target_x:.2f},{target_y:.2f},{target_theta:.1f}°) "
            f"当前:({current_x:.2f},{current_y:.2f},{current_theta:.1f}°) "
            f"误差:距离={dist_error:.3f}m,角度={d_theta:.1f}° "
            f"速度:线={linear_vel:.2f},角={angular_vel:.2f}"
        )

    def handle_waypoint_arrival(self):
        """处理到达路径点后的逻辑"""
        rospy.loginfo(f"到达路径点 {self.current_waypoint_index+1}")
        rospy.loginfo(f"当前速度: 线速度={self.current_velocity['linear']:.3f}, 角速度={self.current_velocity['angular']:.3f}")
        rospy.loginfo(f"停止持续时间: {self.stopped_duration:.1f}秒")
        
        # 对于第一个点，需要基本停止后才开始舞蹈（放宽条件）
        if self.current_waypoint_index == 0 and not self.dance_service_called:
            # 检查是否基本停止（放宽速度阈值和停止时间要求）
            if (self.current_velocity["linear"] < 0.1 and 
                self.current_velocity["angular"] < 0.1 and 
                self.stopped_duration >= self.required_stop_time):
                rospy.loginfo("机器人已基本停止，开始执行舞蹈")
                self.set_state(NavigationState.WAITING)
                self.perform_dance()
            else:
                rospy.loginfo(f"等待机器人基本停止... (当前停止时间: {self.stopped_duration:.1f}s, 需要: {self.required_stop_time}s)")
                # 不改变状态，继续等待
        else:
            # 其他点或舞蹈已调用，直接按等待时间停留
            rospy.loginfo(f"不在第一个点或舞蹈已调用，按等待时间停留")
            self.set_state(NavigationState.WAITING)
            self.schedule_next_waypoint()

    def start_navigation_watchdog(self):
        """Start a timer to periodically check if navigation is progressing"""
        if hasattr(self, 'nav_watchdog_timer') and self.nav_watchdog_timer:
            self.nav_watchdog_timer.shutdown()
        self.nav_watchdog_timer = rospy.Timer(
            rospy.Duration(8.0), self.check_navigation_progress
        )  # 增加检查间隔
        self._timers.append(self.nav_watchdog_timer)

    def ensure_movement(self, event):
        """简化的移动检查 - 减少干预，让底层控制器处理"""
        current_state = self.get_state()
        if current_state in [NavigationState.DANCING, NavigationState.COMPLETED]:
            return

        # 简化的超时检查 - 只在导航状态且停留超过20秒时才干预
        if (current_state == NavigationState.NAVIGATING and 
            self.waypoint_start_time and 
            (rospy.Time.now() - self.waypoint_start_time).to_sec() > 20.0):
            rospy.logwarn(f"在路径点{self.current_waypoint_index+1}停留超过20秒，强制前进")
            self.force_move_to_next_waypoint()

    def force_move_to_next_waypoint(self):
        """强制移动到下一个路径点，不考虑当前点是否到达"""
        with self._lock:
            # 取消当前目标
            cancel_msg = GoalID()
            self.cancel_pub.publish(cancel_msg)
            rospy.sleep(0.1)
            
            self.current_waypoint_index += 1
            rospy.loginfo(f"[调试] 强制前进，当前索引: {self.current_waypoint_index}, 总航点数: {len(self.waypoints)}")
            
            if self.current_waypoint_index < len(self.waypoints):
                rospy.loginfo(
                    f"强制前进到路径点 {self.current_waypoint_index+1}/{len(self.waypoints)}"
                )
                # 短暂延迟后尝试下一个点
                rospy.sleep(1.0)
                self.navigate_to_current_waypoint()
                self.waypoint_start_time = rospy.Time.now()  # 重置计时器
            else:
                rospy.loginfo("[完成] 所有路径点已完成!")
                rospy.loginfo(f"[调试] 最终航点索引: {self.current_waypoint_index}, 总航点数: {len(self.waypoints)}")
                self.reached_final = True
                self.set_state(NavigationState.COMPLETED)
                # 任务完成，自动退出程序
                rospy.loginfo("导航舞蹈任务完成，程序即将退出")
                rospy.signal_shutdown("任务完成")
                import sys
                sys.exit(0)


    def check_navigation_progress(self, event):
        """简化的导航进度检查"""
        current_state = self.get_state()
        if current_state not in [NavigationState.NAVIGATING]:
            return

        # 简化的卡住检测 - 只检查时间，让底层控制器处理移动
        current_time = rospy.Time.now()
        time_diff = (current_time - self.last_position_check["time"]).to_sec()

        # 更新检查时间
        self.last_position_check["time"] = current_time

        # 如果在一个点停留太久，强制前进到下一个点
        if time_diff > 15.0:  # 15秒后强制前进
            rospy.logwarn(f"[超时] 在路径点停留超过{time_diff:.1f}秒，强制前进...")
            self.force_move_to_next_waypoint()
            return


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

    def should_use_direct_control(self):
        """判断是否应该使用直接控制而不是导航"""
        total_waypoints = len(self.waypoints)
        current_index = self.current_waypoint_index
        
        # 第一个点使用导航（确保从后台正确到达舞台）
        if current_index == 0:
            return False
        
        # 最后两个点使用导航（确保正确返回后台）
        if current_index >= total_waypoints - 2:
            return False
            
        # 中间的点使用直接控制
        return True

    def navigate_to_current_waypoint(self, is_retry=False):
        """Navigate to the current waypoint with error handling"""
        with self._lock:
            rospy.loginfo(f"[调试] 检查航点索引: {self.current_waypoint_index} >= {len(self.waypoints)}")
            if self.current_waypoint_index >= len(self.waypoints):
                rospy.loginfo("[完成] 所有路径点已完成!")
                rospy.loginfo(f"[调试] 最终航点索引: {self.current_waypoint_index}, 总航点数: {len(self.waypoints)}")
                self.reached_final = True
                self.set_state(NavigationState.COMPLETED)
                # 任务完成，自动退出程序
                rospy.loginfo("导航舞蹈任务完成，程序即将退出")
                rospy.signal_shutdown("任务完成")
                import sys
                sys.exit(0)
                return

            # Get current waypoint
            x, y, theta = self.waypoints[self.current_waypoint_index]

            # Determine location description
            if self.current_waypoint_index == len(self.waypoints) - 1:
                location_desc = "[返回后台]"
            elif self.current_waypoint_index == 0:
                location_desc = f"[第一个舞蹈位置]"
            else:
                location_desc = f"[舞蹈位置 {self.current_waypoint_index+1}]"

            wait_time = (
                self.wait_times[self.current_waypoint_index]
                if self.current_waypoint_index < len(self.wait_times)
                else 0
            )
            
            # 判断使用哪种控制方式
            use_direct_control = self.should_use_direct_control()
            
            if use_direct_control:
                rospy.loginfo(f"[直接控制] {location_desc} x={x}, y={y}, θ={theta} (等待时间: {wait_time}秒)")
                self.set_state(NavigationState.NAVIGATING)
                self.waypoint_start_time = rospy.Time.now()
                self.start_direct_control(x, y, theta)
                return
            
            # 使用导航系统
            # Reset for new navigation attempt
            if not is_retry:
                self.goal_send_retries = 0

            # First ensure any previous goals are canceled
            cancel_msg = GoalID()
            self.cancel_pub.publish(cancel_msg)
            rospy.sleep(0.1)

            prefix = "[重试] " if is_retry else ""
            rospy.loginfo(
                f"{prefix}[导航目标 {self.current_waypoint_index+1}/{len(self.waypoints)}] {location_desc} x={x}, y={y}, θ={theta} (等待时间: {wait_time}秒)"
            )
            rospy.loginfo(f"[调试] 当前航点索引: {self.current_waypoint_index}, 总航点数: {len(self.waypoints)}")
            rospy.loginfo(f"[调试] 剩余航点: {self.waypoints[self.current_waypoint_index:]}")

            # Update last position check
            self.last_position_check["x"] = self.current_position["x"]
            self.last_position_check["y"] = self.current_position["y"]
            self.last_position_check["time"] = rospy.Time.now()

            # Publish MoveBaseActionGoal
            goal_msg = self._build_move_base_goal(x, y, theta)
            rospy.loginfo(f"发布导航目标: x={x}, y={y}, theta={theta}")
            self.goal_pub.publish(goal_msg)
            self.last_goal_send_time = rospy.Time.now()
            self.set_state(NavigationState.NAVIGATING)
            self.waypoint_start_time = rospy.Time.now()  # 重置路径点计时器
            rospy.loginfo("导航目标已发布，等待路径规划...")
            
            # 添加路径规划失败检测定时器
            self.plan_failure_timer = rospy.Timer(
                rospy.Duration(5.0), self.check_plan_failure, oneshot=True
            )
            self._timers.append(self.plan_failure_timer)

    def check_plan_failure(self, event):
        """检查路径规划是否失败，如果失败则跳过当前点"""
        current_state = self.get_state()
        if current_state == NavigationState.NAVIGATING:
            # 检查机器人是否在移动
            current_time = rospy.Time.now()
            time_since_start = (current_time - self.waypoint_start_time).to_sec()
            
            if time_since_start > 5.0:  # 5秒后检查
                # 检查机器人是否移动了
                dx = self.current_position["x"] - self.last_position_check["x"]
                dy = self.current_position["y"] - self.last_position_check["y"]
                dist_moved = math.hypot(dx, dy)
                
                if dist_moved < 0.1:  # 如果几乎没有移动
                    rospy.logwarn(f"[路径规划失败] 机器人5秒内只移动了{dist_moved:.3f}米，跳过当前点")
                    self.force_move_to_next_waypoint()

    def perform_dance(self):
        """
        Call the dance service, only execute on first call
        """
        if self.dance_service_called:
            rospy.loginfo("舞蹈服务已被调用，不再重复调用")
            return

        if self.play_dance_service is None:
            rospy.logwarn("舞蹈服务不可用，跳过舞蹈")
            self.force_move_to_next_waypoint()
            return

        try:
            # 确保机器人完全停止
            rospy.loginfo("发送停止命令确保机器人完全停止...")
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            rospy.sleep(0.5)  # 等待停止命令生效
            
            self.set_state(NavigationState.DANCING)
            dance_direction = self.dance_type
            rospy.loginfo(
                f"开始舞蹈: {dance_direction}，在此表演中只会调用一次舞蹈服务"
            )
            self.dance_direction_pub.publish(String(dance_direction))
            rospy.sleep(0.5)

            # 在真正的非阻塞方式中调用服务
            dance_thread = threading.Thread(target=self._call_dance_service)
            dance_thread.daemon = True
            dance_thread.start()
            self._threads.append(dance_thread)

            # 标记舞蹈服务已调用
            self.dance_service_called = True
            rospy.loginfo("舞蹈服务调用已启动，等待完成后将继续导航")

            # 设置一个定时器，确保即使舞蹈服务卡住也能继续
            self.schedule_next_waypoint()

        except Exception as e:
            rospy.logerr(f"舞蹈服务调用失败: {e}")
            # 即使舞蹈失败，也应该继续前进
            self.force_move_to_next_waypoint()

    def _call_dance_service(self):
        """Helper method to call dance service in a separate thread"""
        try:
            response = self.play_dance_service()
            if response.success:
                rospy.loginfo("舞蹈服务已完成")
            else:
                rospy.logwarn(f"舞蹈服务返回失败: {response.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"舞蹈服务调用异常: {e}")
        except Exception as e:
            rospy.logerr(f"舞蹈服务执行错误: {e}")

    def schedule_next_waypoint(self):
        """Schedule movement to next waypoint after waiting period"""
        # 减少等待时间，确保不间断移动
      #   wait_time = min(
      #       5.0,
      #       (
      #           self.wait_times[self.current_waypoint_index]
      #           if self.current_waypoint_index < len(self.wait_times)
      #           else 0
      #       ),
      #   )
      #           self.wait_times[self.current_waypoint_index]
      
      
        wait_time=self.wait_times[self.current_waypoint_index]
        
        
        # If we have an existing timer, cancel it
        if self.dance_timer:
            self.dance_timer.shutdown()

        # Schedule the continuation after the wait time
        rospy.loginfo(f"计划在{wait_time}秒后移动到下一个路径点")
        self.dance_timer = rospy.Timer(
            rospy.Duration(wait_time), self.continue_to_next_waypoint, oneshot=True
        )
        self._timers.append(self.dance_timer)

    def continue_to_next_waypoint(self, event=None):
        """Timer callback to continue to next waypoint"""
        rospy.loginfo("等待时间结束，继续前往下一个路径点")

        with self._lock:
            self.set_state(NavigationState.IDLE)
            self.current_waypoint_index += 1
            rospy.loginfo(f"[调试] 前进到下一个航点，当前索引: {self.current_waypoint_index}, 总航点数: {len(self.waypoints)}")

            if self.current_waypoint_index < len(self.waypoints):
                rospy.loginfo(
                    f"移动到路径点 {self.current_waypoint_index+1}/{len(self.waypoints)}"
                )
                self.navigate_to_current_waypoint()
            else:
                rospy.loginfo("[完成] 所有路径点已完成!")
                rospy.loginfo(f"[调试] 最终航点索引: {self.current_waypoint_index}, 总航点数: {len(self.waypoints)}")
                self.reached_final = True
                self.set_state(NavigationState.COMPLETED)
                # 任务完成，自动退出程序
                rospy.loginfo("导航舞蹈任务完成，程序即将退出")
                rospy.signal_shutdown("任务完成")
                import sys
                sys.exit(0)

    def feedback_callback(self, msg):
        """Handle navigation feedback, detect arrival with very loose thresholds"""
        current_state = self.get_state()
        if self.reached_final or current_state in [NavigationState.DANCING, NavigationState.COMPLETED]:
            return

        # Update current position
        current_pose = msg.feedback.base_position.pose
        self.current_position["x"] = current_pose.position.x
        self.current_position["y"] = current_pose.position.y

        orientation = current_pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = tft.euler_from_quaternion(quaternion)
        self.current_position["theta"] = math.degrees(euler[2])

        # 持续打印当前位姿信息
        rospy.loginfo(f"[当前位姿] x={current_pose.position.x:.3f}, y={current_pose.position.y:.3f}, yaw={self.current_position['theta']:.2f}°")
        
        # 如果使用直接控制，不需要处理导航反馈
        if self.direct_control_enabled:
            return

        # Get current waypoint
        x, y, target_yaw = self.waypoints[self.current_waypoint_index]

        # Calculate distance to target
        dx = current_pose.position.x - x
        dy = current_pose.position.y - y
        dist = math.hypot(dx, dy)

        # Calculate angular difference - choose shortest path
        current_yaw = self.current_position["theta"]
        d_yaw = target_yaw - current_yaw  # 修正：目标角度减去当前角度
        
        # Normalize to the range [-180, 180] for shortest path
        while d_yaw > 180:
            d_yaw -= 360
        while d_yaw < -180:
            d_yaw += 360
            
        # Use absolute value for threshold comparison
        d_yaw_abs = abs(d_yaw)
        
        # Log information less frequently
        rospy.loginfo_throttle(
            5,
            f"[导航状态] 距离目标: {dist:.2f} 米, 角度差: {d_yaw:.2f} 度 (最短路径)",
        )

        # 检查是否到达目标点（距离和角度都要满足）
        if (dist <= self.threshold and d_yaw_abs <= self.angle_threshold and current_state == NavigationState.NAVIGATING):
            rospy.loginfo(f"到达路径点 {self.current_waypoint_index+1} (距离:{dist:.2f}m, 角度差:{d_yaw:.2f}度)")
            rospy.loginfo(f"当前速度: 线速度={self.current_velocity['linear']:.3f}, 角速度={self.current_velocity['angular']:.3f}")
            rospy.loginfo(f"停止持续时间: {self.stopped_duration:.1f}秒")
            
            # 对于第一个点，需要基本停止后才开始舞蹈（放宽条件）
            if self.current_waypoint_index == 0 and not self.dance_service_called:
                # 检查是否基本停止（放宽速度阈值和停止时间要求）
                if (self.current_velocity["linear"] < 0.1 and 
                    self.current_velocity["angular"] < 0.1 and 
                    self.stopped_duration >= self.required_stop_time):
                    rospy.loginfo("机器人已基本停止，开始执行舞蹈")
                    self.set_state(NavigationState.WAITING)
                    self.perform_dance()
                else:
                    rospy.loginfo(f"等待机器人基本停止... (当前停止时间: {self.stopped_duration:.1f}s, 需要: {self.required_stop_time}s)")
                    # 不改变状态，继续等待
            else:
                # 其他点或舞蹈已调用，直接按等待时间停留
                rospy.loginfo(f"不在第一个点或舞蹈已调用，按等待时间停留")
                self.set_state(NavigationState.WAITING)
                self.schedule_next_waypoint()
        elif (dist <= self.threshold and current_state == NavigationState.NAVIGATING):
            # 距离到达但角度未到达，继续等待
            rospy.loginfo(f"距离已到达 (距离:{dist:.2f}m)，但角度未到达 (角度差:{d_yaw:.2f}度, 需要:{self.angle_threshold}度)")


if __name__ == "__main__":
    rospy.init_node("simple_nav_waypoints_player")
    parser = argparse.ArgumentParser(
        description="Navigation Dance Performance Controller"
    )
    parser.add_argument(
        "--dance",
        type=str,
        default="A",
        choices=["A", "B", "X", "Y","Up","Down"],
        help="Specify dance type to execute",
    )
    args, unknown = parser.parse_known_args()

    backstage_pos = (-0.6, 0, 0)

    dance_choreography = {
       #祖国的好山河
        "A": [
            ((-2.3, 3.4, 170), 30.0),
            ((-2.4, 3.4, 180), 40.0),
            ((-2.6, 3.4, -170), 20.0),
            ((-0.6, 0, 0), 0),
        ],
        #沙家浜总有一天
        "B": [
            ((-2.1, 3.4, 170), 20.0),
            ((-2.3, 3.4, 180), 40.0),
            ((-2.6, 3.4, -170), 20.0),
            ((-2.8, 3.4, 180), 12.5),
            ((-0.6, 0, 0), 0),
        ],
         #军民鱼水情
        "X": [
            ((-2.4, 2.6, 172), 60.0),
            ((-2.7, 3.2, 160), 50.0),
            ((-2.7, 3.5, 160), 63.5),
            ((-3.1, 3.4, 170), 60.5),
            ((-0.6, 0, 0), 0),
        ],
        #智斗
        "Y": [
            ((-2.2, 3.4, 170), 50.0),
            ((-2.5, 3.4, 180), 40.0),
            ((-2.8, 3.4, -170), 50.0),
            ((-0.6, 0, 0), 0),
        ],
        # 上台
        "Up": [
            ((-2.6, 3.4, 170), 30.0),
            ((-0.6, 0, 0), 0),
            
        ],
        
        #下台
         "Down": [
            ((-2.6, 3.4, 170), 30.0),
            ((-0.6, 0, 0), 0),
        ],
    }
    # 检查是否是Up或Down参数，如果是则直接调用_play_tts_with_action
    if args.dance in ["Up", "Down"]:
        rospy.loginfo(f"检测到{args.dance}参数，直接调用TTS和动作播放")
        
        # 初始化G1ActionPlayer
        try:
            action_player = G1ActionPlayer()
            
            # 根据参数选择对应的TTS文本和动作目录
            if args.dance == "Up":
                tts_text = action_player.tts_presets.get('B', "各位朋友，大家好。在江南水乡沙家浜，曾镌刻下一段军民同心、共抗敌寇的红色记忆。这里有指导员郭建光的壮志凌云，有阿庆嫂的机智沉着，有沙奶奶的慈爱坚毅，也有与敌人周旋的惊心动魄。接下来，让我们循着京剧《沙家浜》的经典旋律，一同穿越烽火岁月，重温那段充满斗争智慧与深厚情谊的历史！")
                action_dir = "start_b"
            else:  # Down
                tts_text = action_player.tts_presets.get('C', "各位朋友，经典的唱腔余韵悠长，烽火里的故事依旧动人。我们刚刚一同重温了郭建光的壮志、沙奶奶的坚韧，也深深记住了阿庆嫂垒起七星灶的过人智慧，更读懂了那份跨越岁月的军民鱼水情。本场沙家浜京剧选段演出到此圆满结束，感谢您的驻足与陪伴，我们下次再会！")
                action_dir = "start_x"
            
            # 调用_play_tts_with_action函数
            action_player._play_tts_with_action(tts_text, action_dir, 0)
            
            rospy.loginfo(f"TTS和动作播放完成: {args.dance}")
            rospy.loginfo("任务完成，程序即将退出")
            
        except Exception as e:
            rospy.logerr(f"调用_play_tts_with_action失败: {e}")
            rospy.loginfo("任务失败，程序即将退出")
        finally:
            # 确保程序退出
            rospy.signal_shutdown("任务完成")
            import sys
            sys.exit(0)
    else:
        # 原有的导航舞蹈逻辑
        node = SimpleNavWaypointPlayer(
            backstage_pos=backstage_pos,
            dance_type=args.dance,
            dance_choreography=dance_choreography,
        )

        rospy.loginfo(f"表演开始，使用舞蹈类型: {args.dance}")
        rospy.spin()
