#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import tf
import tf.transformations as tft
import sys
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import threading
import time

class SimpleWaypointNavigator:
    def __init__(self, waypoints, wait_time=5.0):
        """
        简单的航点导航器
        :param waypoints: 导航点列表 [(x, y, yaw_deg), ...]
        :param wait_time: 每个点停留时间(秒)
        """
        # 初始化ROS节点
        rospy.init_node("simple_waypoint_nav")
        
        self.waypoints = waypoints
        self.wait_time = wait_time
        self.running = True
        
        # 创建发布器
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.marker_pub = rospy.Publisher('/nav_markers', MarkerArray, queue_size=1)
        
        # 创建TF监听器
        self.tf_listener = tf.TransformListener()
        rospy.sleep(0.5)  # 给TF监听器一些时间初始化
        
        # 当前位置
        self.current_pose = None
        
        # 航点索引
        self.current_waypoint_index = 0
        
        # 创建一个单独的线程来运行导航，这样主线程可以处理中断
        self.nav_thread = threading.Thread(target=self.navigation_loop)
        self.nav_thread.daemon = True  # 设置为守护线程，这样当主线程退出时，这个线程也会退出
        
        # 记录是否已经完成导航
        self.navigation_complete = False
        
        rospy.loginfo("简单航点导航器初始化完成")
    
    def get_robot_position(self):
        """获取机器人在map坐标系中的位置"""
        try:
            # 等待TF树中从map到base_link的转换可用
            if self.tf_listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(1.0)):
                # 获取转换
                (trans, rot) = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
                
                # 创建一个Pose对象
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = trans[0]
                pose.pose.position.y = trans[1]
                pose.pose.position.z = trans[2]
                pose.pose.orientation.x = rot[0]
                pose.pose.orientation.y = rot[1]
                pose.pose.orientation.z = rot[2]
                pose.pose.orientation.w = rot[3]
                
                self.current_pose = pose
                return True
            else:
                rospy.logwarn("无法获取从map到base_link的转换")
                return False
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"获取机器人位置时出错: {e}")
            return False
    
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
    
    def publish_markers(self):
        """发布当前位置和所有航点的标记"""
        marker_array = MarkerArray()
        
        # 添加当前位置标记 (红色球体)
        if self.current_pose:
            current_marker = Marker()
            current_marker.header.frame_id = "map"
            current_marker.header.stamp = rospy.Time.now()
            current_marker.ns = "navigation"
            current_marker.id = 0
            current_marker.type = Marker.SPHERE
            current_marker.action = Marker.ADD
            current_marker.pose = self.current_pose.pose
            current_marker.scale.x = 0.3
            current_marker.scale.y = 0.3
            current_marker.scale.z = 0.3
            current_marker.color.r = 1.0
            current_marker.color.g = 0.0
            current_marker.color.b = 0.0
            current_marker.color.a = 1.0
            current_marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(current_marker)
            
            # 添加当前位置文本标记
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "navigation"
            text_marker.id = 100
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = self.current_pose.pose
            text_marker.pose.position.z += 0.4
            text_marker.text = "当前位置"
            text_marker.scale.z = 0.2
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(text_marker)
        
        # 添加所有航点标记
        for i, wp in enumerate(self.waypoints):
            x, y, yaw = wp
            
            # 航点标记 (绿色箭头)
            arrow = Marker()
            arrow.header.frame_id = "map"
            arrow.header.stamp = rospy.Time.now()
            arrow.ns = "navigation"
            arrow.id = i + 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            
            # 设置位姿
            arrow.pose = self.create_pose_stamped(x, y, yaw).pose
            
            # 设置大小
            arrow.scale.x = 0.5  # 长度
            arrow.scale.y = 0.1  # 宽度
            arrow.scale.z = 0.1  # 高度
            
            # 设置颜色 (绿色，当前目标点为黄色)
            if i == self.current_waypoint_index:
                arrow.color.r = 1.0
                arrow.color.g = 1.0
                arrow.color.b = 0.0  # 黄色
            else:
                arrow.color.r = 0.0
                arrow.color.g = 1.0
                arrow.color.b = 0.0  # 绿色
            arrow.color.a = 1.0
            
            arrow.lifetime = rospy.Duration(0)
            marker_array.markers.append(arrow)
            
            # 航点文本标记
            text = Marker()
            text.header.frame_id = "map"
            text.header.stamp = rospy.Time.now()
            text.ns = "navigation"
            text.id = i + 200
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose = self.create_pose_stamped(x, y, yaw).pose
            text.pose.position.z += 0.3
            text.text = f"航点 {i+1}: ({x:.2f}, {y:.2f})"
            text.scale.z = 0.2
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.lifetime = rospy.Duration(0)
            marker_array.markers.append(text)
        
        # 发布标记数组
        self.marker_pub.publish(marker_array)
    
    def send_goal(self, waypoint):
        """发送导航目标"""
        x, y, yaw = waypoint
        goal = self.create_pose_stamped(x, y, yaw)
        
        # 发布目标
        self.goal_pub.publish(goal)
        rospy.loginfo(f"发送导航目标: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}°")
        
        # 更新标记
        self.publish_markers()
    
    def navigation_loop(self):
        """导航循环，在单独的线程中运行"""
        rospy.loginfo("开始航点导航")
        
        while self.running and not rospy.is_shutdown() and self.current_waypoint_index < len(self.waypoints):
            # 获取当前位置
            if not self.get_robot_position():
                rospy.logwarn("无法获取机器人位置，等待1秒后重试")
                time.sleep(1)
                continue
            
            # 发布所有标记
            self.publish_markers()
            
            # 获取当前航点
            waypoint = self.waypoints[self.current_waypoint_index]
            
            # 发送目标
            self.send_goal(waypoint)
            
            # 计算到目标的距离
            if self.current_pose:
                x, y, _ = waypoint
                dx = x - self.current_pose.pose.position.x
                dy = y - self.current_pose.pose.position.y
                distance = math.sqrt(dx*dx + dy*dy)
                
                # 估计到达时间 (假设速度为0.5m/s)
                estimated_time = max(distance / 0.5 * 1.5, 5.0)  # 至少5秒
                rospy.loginfo(f"距离目标: {distance:.2f}米，预计需要 {estimated_time:.1f}秒")
                
                # 等待估计的时间
                wait_start = time.time()
                while time.time() - wait_start < estimated_time and self.running and not rospy.is_shutdown():
                    time.sleep(0.1)  # 小的睡眠时间，使线程能响应终止请求
                    
                    # 定期更新位置和标记
                    if time.time() - wait_start > 1.0:  # 每秒更新一次
                        self.get_robot_position()
                        self.publish_markers()
                        
                        # 重新计算距离
                        dx = x - self.current_pose.pose.position.x
                        dy = y - self.current_pose.pose.position.y
                        new_distance = math.sqrt(dx*dx + dy*dy)
                        
                        # 如果已经足够接近目标，就认为到达了
                        if new_distance < 0.3:  # 30厘米以内
                            rospy.loginfo(f"已到达航点 {self.current_waypoint_index+1}")
                            break
            else:
                # 如果没有位置信息，等待固定时间
                time.sleep(10)
            
            # 到达后，在航点停留指定时间
            rospy.loginfo(f"在航点 {self.current_waypoint_index+1} 停留 {self.wait_time}秒")
            wait_start = time.time()
            while time.time() - wait_start < self.wait_time and self.running and not rospy.is_shutdown():
                time.sleep(0.1)
            
            # 移动到下一个航点
            self.current_waypoint_index += 1
            
            # 如果已经完成所有航点，设置完成标志
            if self.current_waypoint_index >= len(self.waypoints):
                self.navigation_complete = True
                rospy.loginfo("所有航点导航完成！")
        
        if not self.running:
            rospy.loginfo("导航被用户中断")
    
    def start(self):
        """开始导航"""
        self.nav_thread.start()
        
        try:
            # 主线程等待，可以响应Ctrl+C
            while not rospy.is_shutdown() and not self.navigation_complete and self.running:
                rospy.sleep(0.5)
        except KeyboardInterrupt:
            rospy.loginfo("接收到中断信号，停止导航")
            self.running = False
            
        # 等待导航线程结束
        if self.nav_thread.is_alive():
            self.nav_thread.join(timeout=1.0)
        
        rospy.loginfo("导航程序已退出")

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
        navigator = SimpleWaypointNavigator(waypoints, wait_time)
        navigator.start()
        
    except Exception as e:
        rospy.logerr(f"发生错误: {e}")