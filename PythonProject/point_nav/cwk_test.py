#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import os
import threading
import tf.transformations as tft
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
# from playsound import playsound


class NavWaypointPlayer:
    def __init__(self, waypoints, dance_directions=None, threshold=0.5):
        """
        使用航点列表初始化。
        每个航点是一个(x, y, theta)元组
        
        参数:
            waypoints: (x, y, theta)元组的列表
            dance_directions: 在每个航点执行的舞蹈方向列表(可选)
            threshold: 认为到达航点的距离阈值(米)
        """
        self.waypoints = waypoints
        self.threshold = threshold
        self.current_waypoint_index = 0
        self.reached_final = False
        self.waiting_time = 2.0  # 航点间等待的秒数
        
        # 舞蹈配置
        self.dance_directions = dance_directions or ['A'] * len(waypoints)  # 如果未指定则使用默认舞蹈
        self.dance_in_progress = False
        
        # 发布导航目标（MoveBaseActionGoal）
        self.goal_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=1)
        # 订阅 /move_base/feedback 获取当前位姿
        self.feedback_sub = rospy.Subscriber("/move_base/feedback", MoveBaseActionFeedback, self.feedback_callback)
        
        # 舞蹈服务客户端
        rospy.loginfo("等待舞蹈服务...")
        self.dance_direction_pub = rospy.Publisher('dance_direction', String, queue_size=1)
        try:
            rospy.wait_for_service('play_dance', timeout=5.0)
            self.play_dance_service = rospy.ServiceProxy('play_dance', Trigger)
            rospy.loginfo("舞蹈服务已连接")
        except rospy.ROSException:
            rospy.logwarn("舞蹈服务不可用，将不执行舞蹈动作")
            self.play_dance_service = None
        
        rospy.sleep(1.0)  # 等待topic连接
        self.navigate_to_current_waypoint()

    def navigate_to_current_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints):
            rospy.loginfo("[完成] 所有航点已完成!")
            self.reached_final = True
            # rospy.signal_shutdown("任务完成")
            return

        x, y, theta = self.waypoints[self.current_waypoint_index]
        
        goal = MoveBaseActionGoal()
        goal.goal.target_pose.header.frame_id = "map"
        goal.goal.target_pose.header.stamp = rospy.Time.now()
        goal.goal.target_pose.pose.position.x = x
        goal.goal.target_pose.pose.position.y = y
        q = tft.quaternion_from_euler(0, 0, math.radians(theta))  # 将角度转换为弧度
        goal.goal.target_pose.pose.orientation.x = q[0]
        goal.goal.target_pose.pose.orientation.y = q[1]
        goal.goal.target_pose.pose.orientation.z = q[2]
        goal.goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo(f"[导航目标 {self.current_waypoint_index+1}/{len(self.waypoints)}] x={x}, y={y}, θ={theta}")
        self.goal_pub.publish(goal)

    def perform_dance(self):
        """调用舞蹈服务执行舞蹈动作"""
        if self.play_dance_service is None:
            rospy.logwarn("舞蹈服务不可用，跳过舞蹈")
            return True
        
        try:
            # 获取当前航点的舞蹈方向
            if self.current_waypoint_index < len(self.dance_directions):
                dance_direction = self.dance_directions[self.current_waypoint_index]
            else:
                dance_direction = 'A'  # 默认舞蹈
            
            rospy.loginfo(f"开始执行舞蹈: {dance_direction}")
            
            self.dance_direction_pub.publish(String(dance_direction))
            rospy.sleep(0.5)  # 等待方向设置生效
            
            # 调用舞蹈服务
            response = self.play_dance_service()
            
            if response.success:
                rospy.loginfo(f"舞蹈执行成功: {response.message}")
                return True
            else:
                rospy.logwarn(f"舞蹈执行失败: {response.message}")
                return False
                
        except rospy.ServiceException as e:
            rospy.logerr(f"舞蹈服务调用失败: {e}")
            return False

    def feedback_callback(self, msg):
        if self.reached_final or self.dance_in_progress:
            return

        current_pose = msg.feedback.base_position.pose
        x, y, _ = self.waypoints[self.current_waypoint_index]
        dx = current_pose.position.x - x
        dy = current_pose.position.y - y
        dist = math.hypot(dx, dy)  # 欧氏距离

        rospy.loginfo_throttle(2, f"[当前位置] ({current_pose.position.x:.2f}, {current_pose.position.y:.2f}) -> 距离目标 {dist:.2f} m")

        if dist <= self.threshold:
            rospy.loginfo(f"[到达航点 {self.current_waypoint_index+1}/{len(self.waypoints)}]")
            
            # 在此航点执行舞蹈
            self.dance_in_progress = True
            
            # 创建一个线程来执行舞蹈，这样不会阻塞反馈处理
            dance_thread = threading.Thread(target=self._execute_dance_and_continue)
            dance_thread.daemon = True
            dance_thread.start()

    def _execute_dance_and_continue(self):
        """在单独的线程中执行舞蹈，然后继续导航"""
        try:
            # 执行舞蹈动作
            rospy.loginfo("/* 开始执行指定舞蹈动作... */")
            dance_success = self.perform_dance()
            rospy.loginfo("/* 舞蹈动作执行完毕 */")
            
            # 前往下一个航点
            self.current_waypoint_index += 1
            
            # 等待一段时间后继续
            if self.current_waypoint_index < len(self.waypoints):
                rospy.loginfo(f"等待 {self.waiting_time} 秒后前往下一个航点...")
                rospy.sleep(self.waiting_time)
                self.navigate_to_current_waypoint()
            else:
                rospy.loginfo("[完成] 所有航点已完成!")
                self.reached_final = True
                # rospy.signal_shutdown("任务完成")
            
        except Exception as e:
            rospy.logerr(f"舞蹈执行线程错误: {e}")
        finally:
            self.dance_in_progress = False


if __name__ == "__main__":
    rospy.init_node("nav_waypoints_player")

    # 每个航点为 (x, y, theta) 其中 theta 为角度制
    waypoints = [
        (0.18, -0.7,-179),   #后台位置
        (4.18, 1.15,-159),   #舞台的第一个点位   开始舞蹈
        
        (-5.238, -0.204, 117.677), #  舞蹈的第二个点位
        
        
        (1.122, -0.413, -53.2),  # 舞蹈的第三个点位、
        
        
        (0.18, -0.7,-179),   #  回到后台
    ]
    
    # 可选: 'Up', 'Down', 'Left', 'Right', 'A', 'B', 'X', 'Y'
    dance_directions = [
        'A',           # 第1个航点执行 'A' 舞蹈
        'B',           # 第2个航点执行 'B' 舞蹈
        'Up',          # 第3个航点执行 'Up' 舞蹈
    ]
    
    threshold = 0.5  # 到达目标点的距离阈值（米）

    node = NavWaypointPlayer(waypoints, dance_directions, threshold)
    rospy.spin()