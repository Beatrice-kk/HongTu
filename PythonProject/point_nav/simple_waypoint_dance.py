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

class SimpleWaypointDance:
    def __init__(self, waypoints, dance_type="A"):
        """
        初始化简单的航点舞蹈控制器
        waypoints: [(x, y, theta, wait_time), ...] 航点列表
        dance_type: 舞蹈类型
        """
        rospy.init_node("simple_waypoint_dance")
        
        # 航点信息
        self.waypoints = waypoints
        self.dance_type = dance_type
        self.current_waypoint_index = 0
        self._state = NavigationState.IDLE
        
        # 当前位置跟踪
        self.current_position = {"x": 0.0, "y": 0.0, "theta": 0.0}
        
        # 舞蹈状态
        self.dance_service_called = False
        
        # 到达阈值
        self.threshold = 0.5  # 距离阈值：0.5米
        self.angle_threshold = 30.0  # 角度阈值：30度
        
        # 发布器和订阅器
        self.goal_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=1)
        self.feedback_sub = rospy.Subscriber("/move_base/feedback", MoveBaseActionFeedback, self.feedback_callback)
        self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self.dance_direction_pub = rospy.Publisher("dance_direction", String, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        # 舞蹈服务客户端
        try:
            rospy.wait_for_service("play_dance", timeout=5.0)
            self.play_dance_service = rospy.ServiceProxy("play_dance", Trigger)
            rospy.loginfo("舞蹈服务连接成功")
        except rospy.ROSException:
            rospy.logwarn("舞蹈服务不可用")
            self.play_dance_service = None
        
        # 定时器
        self.wait_timer = None
        
        rospy.loginfo(f"初始化完成，共{len(self.waypoints)}个航点")
        rospy.loginfo(f"航点列表: {self.waypoints}")
        
        # 开始导航到第一个航点
        self.navigate_to_current_waypoint()
    
    def navigate_to_current_waypoint(self):
        """导航到当前航点"""
        if self.current_waypoint_index >= len(self.waypoints):
            rospy.loginfo("所有航点已完成!")
            self._state = NavigationState.COMPLETED
            rospy.signal_shutdown("任务完成")
            return
        
        # 获取当前航点信息
        x, y, theta, wait_time = self.waypoints[self.current_waypoint_index]
        
        rospy.loginfo(f"导航到航点 {self.current_waypoint_index+1}/{len(self.waypoints)}: ({x}, {y}, {theta}°) 等待时间: {wait_time}秒")
        
        # 取消之前的目标
        cancel_msg = GoalID()
        self.cancel_pub.publish(cancel_msg)
        rospy.sleep(0.1)
        
        # 发布新目标
        goal_msg = self._build_move_base_goal(x, y, theta)
        self.goal_pub.publish(goal_msg)
        self._state = NavigationState.NAVIGATING
        
        rospy.loginfo("导航目标已发布")
    
    def _build_move_base_goal(self, x, y, theta_deg):
        """构建导航目标消息"""
        goal = MoveBaseActionGoal()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.goal_id.stamp = rospy.Time.now()
        goal.goal_id.id = f"waypoint_{self.current_waypoint_index}_{rospy.Time.now().to_nsec()}"
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
    
    def feedback_callback(self, msg):
        """处理导航反馈"""
        if self._state != NavigationState.NAVIGATING:
            return
        
        # 更新当前位置
        current_pose = msg.feedback.base_position.pose
        self.current_position["x"] = current_pose.position.x
        self.current_position["y"] = current_pose.position.y
        
        orientation = current_pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = tft.euler_from_quaternion(quaternion)
        self.current_position["theta"] = math.degrees(euler[2])
        
        # 获取当前航点
        x, y, target_yaw, wait_time = self.waypoints[self.current_waypoint_index]
        
        # 计算距离和角度差
        dx = current_pose.position.x - x
        dy = current_pose.position.y - y
        dist = math.hypot(dx, dy)
        
        current_yaw = self.current_position["theta"]
        d_yaw = target_yaw - current_yaw  # 修正后的角度误差计算
        while d_yaw > 180:
            d_yaw -= 360
        while d_yaw < -180:
            d_yaw += 360
        d_yaw_abs = abs(d_yaw)
        
        rospy.loginfo_throttle(3, f"当前位置: ({current_pose.position.x:.2f}, {current_pose.position.y:.2f}, {self.current_position['theta']:.1f}°) -> 目标: ({x}, {y}, {target_yaw}°) 距离: {dist:.2f}m, 角度差: {d_yaw:.1f}°")
        
        # 检查是否到达目标
        if dist <= self.threshold and d_yaw_abs <= self.angle_threshold:
            rospy.loginfo(f"到达航点 {self.current_waypoint_index+1}")
            
            # 如果是第一个航点且还没调用舞蹈，则调用舞蹈
            if self.current_waypoint_index == 0 and not self.dance_service_called:
                rospy.loginfo("到达第一个航点，开始舞蹈")
                self._state = NavigationState.DANCING
                self.perform_dance()
            else:
                # 其他航点，直接等待指定时间
                rospy.loginfo(f"在航点 {self.current_waypoint_index+1} 等待 {wait_time} 秒")
                self._state = NavigationState.WAITING
                self.schedule_next_waypoint(wait_time)
    
    def perform_dance(self):
        """执行舞蹈"""
        if self.dance_service_called:
            rospy.loginfo("舞蹈已调用过，跳过")
            return
        
        if self.play_dance_service is None:
            rospy.logwarn("舞蹈服务不可用，跳过舞蹈")
            self.dance_service_called = True
            # 获取等待时间并继续
            _, _, _, wait_time = self.waypoints[self.current_waypoint_index]
            self.schedule_next_waypoint(wait_time)
            return
        
        try:
            # 确保机器人停止
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            rospy.sleep(0.5)
            
            rospy.loginfo(f"开始舞蹈: {self.dance_type}")
            self.dance_direction_pub.publish(String(self.dance_type))
            rospy.sleep(0.5)
            
            # 在单独线程中调用舞蹈服务
            dance_thread = threading.Thread(target=self._call_dance_service)
            dance_thread.daemon = True
            dance_thread.start()
            
            self.dance_service_called = True
            
            # 获取等待时间并继续
            _, _, _, wait_time = self.waypoints[self.current_waypoint_index]
            self.schedule_next_waypoint(wait_time)
            
        except Exception as e:
            rospy.logerr(f"舞蹈调用失败: {e}")
            self.dance_service_called = True
            _, _, _, wait_time = self.waypoints[self.current_waypoint_index]
            self.schedule_next_waypoint(wait_time)
    
    def _call_dance_service(self):
        """在单独线程中调用舞蹈服务"""
        try:
            response = self.play_dance_service()
            if response.success:
                rospy.loginfo("舞蹈服务完成")
            else:
                rospy.logwarn(f"舞蹈服务失败: {response.message}")
        except Exception as e:
            rospy.logerr(f"舞蹈服务异常: {e}")
    
    def schedule_next_waypoint(self, wait_time):
        """安排下一个航点"""
        if self.wait_timer:
            self.wait_timer.shutdown()
        
        rospy.loginfo(f"等待 {wait_time} 秒后前往下一个航点")
        self.wait_timer = rospy.Timer(rospy.Duration(wait_time), self.continue_to_next_waypoint, oneshot=True)
    
    def continue_to_next_waypoint(self, event=None):
        """继续到下一个航点"""
        rospy.loginfo("等待时间结束，前往下一个航点")
        
        self.current_waypoint_index += 1
        self._state = NavigationState.IDLE
        
        if self.current_waypoint_index < len(self.waypoints):
            self.navigate_to_current_waypoint()
        else:
            rospy.loginfo("所有航点已完成!")
            self._state = NavigationState.COMPLETED
            rospy.signal_shutdown("任务完成")

def main():
    parser = argparse.ArgumentParser(description="简单航点舞蹈控制器")
    parser.add_argument("--dance", type=str, default="A", 
                       choices=["A", "B", "X", "Y", "Up", "Down"],
                       help="舞蹈类型")
    args = parser.parse_known_args()[0]
    
    # 定义航点 (x, y, theta, wait_time)
    waypoints = [
        (-2.3, 3.4, 170, 30.0),  # 第一个航点，等待30秒
        (-2.4, 3.4, 180, 40.0),  # 第二个航点，等待40秒
        (-2.6, 3.4, -170, 20.0), # 第三个航点，等待20秒
        (-0.6, 0, 0, 0),         # 返回后台
    ]
    
    rospy.loginfo(f"开始简单航点舞蹈，舞蹈类型: {args.dance}")
    rospy.loginfo(f"航点数量: {len(waypoints)}")
    
    # 创建控制器
    controller = SimpleWaypointDance(waypoints, args.dance)
    
    # 运行
    rospy.spin()

if __name__ == "__main__":
    main()
