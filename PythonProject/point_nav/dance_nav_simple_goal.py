#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import tf.transformations as tft
import numpy as np
import signal
import sys
import actionlib
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback
from geometry_msgs.msg import PoseStamped, Pose
from visualization_msgs.msg import Marker, MarkerArray
from typing import Optional

class GlobalPositionNavigation:
    def __init__(self, waypoints, wait_time=5.0):
        """
        初始化导航器，使用move_base feedback获取真实位置
        :param waypoints: 导航点列表，每个点为 (x, y, yaw_deg) 元组
        :param wait_time: 每个点停留时间(秒)
        """
        rospy.init_node("global_position_navigation")
        self.waypoints = waypoints
        self.wait_time = wait_time
        self.should_shutdown = False
        
        # 设置信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # 初始化位置变量
        self.slam_pose = None
        self.global_pose = None
        
        # 订阅slam_odom话题获取相对位置
        self.odom_sub = rospy.Subscriber('/slam_odom', Odometry, self.odom_callback, queue_size=1)
        
        # 初始化move_base action客户端，用于获取真实全局位置
        self.mb_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("等待move_base action服务器...")
        if self.mb_client.wait_for_server(rospy.Duration(5.0)):
            rospy.loginfo("已连接到move_base action服务器")
        else:
            rospy.logwarn("无法连接到move_base action服务器，将只使用slam_odom位置")
        
        # 创建目标发布器 (用于可视化)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        
        # 创建可视化标记发布器
        self.marker_pub = rospy.Publisher('/waypoint_markers', MarkerArray, queue_size=10)
        rospy.loginfo("已创建标记发布器")
        
        # 等待获取初始位置
        rospy.loginfo("等待位置数据...")
        timeout = rospy.Duration(5.0)
        start_time = rospy.Time.now()
        
        while self.slam_pose is None and not rospy.is_shutdown() and (rospy.Time.now() - start_time < timeout):
            rospy.sleep(0.1)
            
        if self.slam_pose is None:
            rospy.logwarn("未能获取slam_odom数据，将使用默认位置 (0,0,0)")
            self.slam_pose = Pose()
            self.slam_pose.orientation.w = 1.0
        
        # 尝试获取全局位置
        self.update_global_position()
        
        # 发布初始位置标记
        self.publish_position_markers()
        
        rospy.loginfo("导航器初始化完成")
    
    def signal_handler(self, sig, frame):
        """处理Ctrl+C信号"""
        rospy.loginfo("接收到中断信号，正在停止导航...")
        self.should_shutdown = True
        rospy.signal_shutdown("用户中断")
        sys.exit(0)
    
    def odom_callback(self, msg: Odometry):
        """slam_odom回调函数，更新相对位置"""
        self.slam_pose = msg.pose.pose
    
    def update_global_position(self):
        """通过发送空目标获取move_base的全局位置反馈"""
        if not hasattr(self, 'mb_client') or self.mb_client is None:
            return False
            
        # 创建一个虚拟目标，位置与当前slam位置相同
        # 目的仅仅是获取feedback，而不是真的移动机器人
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        if self.slam_pose:
            goal.target_pose.pose = self.slam_pose
        else:
            # 默认位置
            goal.target_pose.pose.position.x = 0
            goal.target_pose.pose.position.y = 0
            goal.target_pose.pose.orientation.w = 1.0
        
        # 设置回调函数来接收反馈
        self.mb_client.send_goal(
            goal,
            feedback_cb=self.move_base_feedback_cb
        )
        
        # 给反馈回调一些时间来执行
        timeout = rospy.Duration(1.0)
        start_time = rospy.Time.now()
        while self.global_pose is None and (rospy.Time.now() - start_time < timeout):
            rospy.sleep(0.1)
        
        # 取消目标，我们只想要反馈
        self.mb_client.cancel_goal()
        
        return self.global_pose is not None
    
    def move_base_feedback_cb(self, feedback: MoveBaseFeedback):
        """move_base反馈回调，更新全局位置"""
        self.global_pose = feedback.base_position.pose
        rospy.loginfo(f"收到全局位置反馈: x={self.global_pose.position.x:.2f}, y={self.global_pose.position.y:.2f}")
    
    def create_pose_stamped(self, x, y, yaw_deg):
        """创建PoseStamped消息"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # 将角度转换为四元数
        q = tft.quaternion_from_euler(0, 0, math.radians(yaw_deg))
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        
        return pose
    
    def publish_position_markers(self):
        """发布显示slam位置和全局位置的标记"""
        marker_array = MarkerArray()
        
        # SLAM位置标记 (蓝色球体)
        if self.slam_pose:
            slam_marker = Marker()
            slam_marker.header.frame_id = "map"
            slam_marker.header.stamp = rospy.Time.now()
            slam_marker.ns = "robot_positions"
            slam_marker.id = 1
            slam_marker.type = Marker.SPHERE
            slam_marker.action = Marker.ADD
            slam_marker.pose = self.slam_pose
            slam_marker.scale.x = 0.3
            slam_marker.scale.y = 0.3
            slam_marker.scale.z = 0.3
            slam_marker.color.r = 0.0
            slam_marker.color.g = 0.0
            slam_marker.color.b = 1.0
            slam_marker.color.a = 1.0
            slam_marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(slam_marker)
            
            # 添加文本标记说明这是SLAM位置
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "robot_positions"
            text_marker.id = 3
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = self.slam_pose
            text_marker.pose.position.z += 0.4  # 文本显示在球体上方
            text_marker.text = "SLAM位置"
            text_marker.scale.z = 0.2  # 文本大小
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(text_marker)
        
        # 全局位置标记 (红色球体)
        if self.global_pose:
            global_marker = Marker()
            global_marker.header.frame_id = "map"
            global_marker.header.stamp = rospy.Time.now()
            global_marker.ns = "robot_positions"
            global_marker.id = 2
            global_marker.type = Marker.SPHERE
            global_marker.action = Marker.ADD
            global_marker.pose = self.global_pose
            global_marker.scale.x = 0.3
            global_marker.scale.y = 0.3
            global_marker.scale.z = 0.3
            global_marker.color.r = 1.0
            global_marker.color.g = 0.0
            global_marker.color.b = 0.0
            global_marker.color.a = 1.0
            global_marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(global_marker)
            
            # 添加文本标记说明这是全局位置
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "robot_positions"
            text_marker.id = 4
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = self.global_pose
            text_marker.pose.position.z += 0.4  # 文本显示在球体上方
            text_marker.text = "全局位置"
            text_marker.scale.z = 0.2  # 文本大小
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(text_marker)
        
        # 发布标记数组
        if marker_array.markers:
            self.marker_pub.publish(marker_array)
            rospy.loginfo("已发布位置标记")
    
    def navigate_to_waypoint(self, waypoint):
        """使用action客户端导航到指定航点"""
        x, y, yaw = waypoint
        
        # 更新当前位置
        self.update_global_position()
        self.publish_position_markers()
        
        # 创建导航目标
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = tft.quaternion_from_euler(0, 0, math.radians(yaw))
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        
        # 同时发布到move_base_simple/goal用于可视化
        pose_stamped = PoseStamped()
        pose_stamped.header = goal.target_pose.header
        pose_stamped.pose = goal.target_pose.pose
        self.goal_pub.publish(pose_stamped)
        
        # 发布目标点标记
        self.publish_waypoint_marker(pose_stamped)
        
        # 计算到目标的距离
        distance = 0
        if self.global_pose:
            dx = x - self.global_pose.position.x
            dy = y - self.global_pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)
            rospy.loginfo(f"到目标点距离: {distance:.2f}米")
        
        # 发送导航目标
        rospy.loginfo(f"导航到目标点: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}°")
        self.mb_client.send_goal(goal)
        
        # 等待导航完成或被取消
        # 设置一个合理的超时时间，基于距离
        timeout = max(60, distance * 5)  # 至少60秒，或者每米5秒
        rospy.loginfo(f"等待导航完成，超时时间: {timeout:.1f}秒")
        
        # 使用waitForResult的同时检查中断标志
        start_time = rospy.Time.now().to_sec()
        while (rospy.Time.now().to_sec() - start_time) < timeout and not self.should_shutdown:
            if self.mb_client.wait_for_result(rospy.Duration(0.5)):
                break
        
        if self.should_shutdown:
            self.mb_client.cancel_goal()
            return False
        
        # 检查结果
        state = self.mb_client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("✅ 成功到达目标点")
            return True
        else:
            rospy.logwarn(f"❌ 导航失败，状态码: {state}")
            return False
    
    def publish_waypoint_marker(self, pose_stamped):
        """发布航点标记"""
        marker_array = MarkerArray()
        
        # 箭头标记显示方向
        arrow = Marker()
        arrow.header = pose_stamped.header
        arrow.ns = "waypoints"
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose = pose_stamped.pose
        arrow.scale.x = 0.5  # 箭头长度
        arrow.scale.y = 0.1  # 箭头宽度
        arrow.scale.z = 0.1  # 箭头高度
        arrow.color.r = 0.0
        arrow.color.g = 1.0  # 绿色
        arrow.color.b = 0.0
        arrow.color.a = 1.0
        arrow.lifetime = rospy.Duration(0)
        marker_array.markers.append(arrow)
        
        # 文本标记显示坐标
        text = Marker()
        text.header = pose_stamped.header
        text.ns = "waypoints"
        text.id = 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose = pose_stamped.pose
        text.pose.position.z += 0.3  # 文本显示在箭头上方
        text.text = f"({pose_stamped.pose.position.x:.2f}, {pose_stamped.pose.position.y:.2f})"
        text.scale.z = 0.2  # 文本大小
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.lifetime = rospy.Duration(0)
        marker_array.markers.append(text)
        
        self.marker_pub.publish(marker_array)
    
    def run_navigation(self):
        """按顺序导航到所有航点"""
        rospy.loginfo("==== 开始执行航点导航 ====")
        
        # 开始前更新并显示当前位置
        self.update_global_position()
        self.publish_position_markers()
        
        if self.global_pose:
            rospy.loginfo(f"机器人全局位置: x={self.global_pose.position.x:.2f}, y={self.global_pose.position.y:.2f}")
        if self.slam_pose:
            rospy.loginfo(f"机器人SLAM位置: x={self.slam_pose.position.x:.2f}, y={self.slam_pose.position.y:.2f}")
        
        for i, waypoint in enumerate(self.waypoints):
            if self.should_shutdown:
                break
                
            rospy.loginfo(f"===== 导航到航点 {i+1}/{len(self.waypoints)} =====")
            success = self.navigate_to_waypoint(waypoint)
            
            if success:
                # 到达后停留指定时间
                rospy.loginfo(f"在航点 {i+1} 停留 {self.wait_time} 秒")
                
                # 分段等待，以便能够响应中断
                start_time = rospy.Time.now().to_sec()
                while (rospy.Time.now().to_sec() - start_time) < self.wait_time and not self.should_shutdown:
                    rospy.sleep(0.1)
            
            # 更新当前位置
            self.update_global_position()
            self.publish_position_markers()
            
            if self.should_shutdown:
                break
        
        if not self.should_shutdown:
            rospy.loginfo("✅ 所有航点导航完成！")
        else:
            rospy.loginfo("❌ 导航被用户中断")

if __name__ == "__main__":
    try:
        # 定义航点: (x, y, yaw角度)
        waypoints = [
            (-5.238, -0.204, 117.677),  # 航点1
            (1.122, -0.413, -53.2),     # 航点2
            (0.0, 0.0, 0.0),            # 航点3 - 回到原点
        ]
        
        # 每个点停留5秒
        wait_time = 5.0
        
        # 创建导航器并开始导航
        navigator = GlobalPositionNavigation(waypoints, wait_time)
        navigator.run_navigation()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断")