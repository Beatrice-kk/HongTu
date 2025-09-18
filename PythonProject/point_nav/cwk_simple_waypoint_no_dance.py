#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import os
import threading
import tf.transformations as tft
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback
from std_msgs.msg import Header
# from playsound import playsound


class NavWaypointPlayer:
    def __init__(self, waypoints, threshold=0.5):
        """
        Initialize with a list of waypoints.
        Each waypoint is a tuple of (x, y, theta)
        """
        self.waypoints = waypoints
        self.threshold = threshold
        self.current_waypoint_index = 0
        self.reached_final = False
        self.waiting_time = 2.0  # Seconds to wait between waypoints

        # 发布导航目标（MoveBaseActionGoal）
        self.goal_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=1)
        # 订阅 /move_base/feedback 获取当前位姿
        self.feedback_sub = rospy.Subscriber("/move_base/feedback", MoveBaseActionFeedback, self.feedback_callback)

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
        q = tft.quaternion_from_euler(0, 0, math.radians(theta))  # Convert degrees to radians
        goal.goal.target_pose.pose.orientation.x = q[0]
        goal.goal.target_pose.pose.orientation.y = q[1]
        goal.goal.target_pose.pose.orientation.z = q[2]
        goal.goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo(f"[导航目标 {self.current_waypoint_index+1}/{len(self.waypoints)}] x={x}, y={y}, θ={theta}")
        self.goal_pub.publish(goal)

    def feedback_callback(self, msg):
        if self.reached_final:
            return

        current_pose = msg.feedback.base_position.pose
        x, y, _ = self.waypoints[self.current_waypoint_index]
        dx = current_pose.position.x - x
        dy = current_pose.position.y - y
        dist = math.hypot(dx, dy)  # 欧氏距离

        rospy.loginfo_throttle(2, f"[当前位置] ({current_pose.position.x:.2f}, {current_pose.position.y:.2f}) -> 距离目标 {dist:.2f} m")

        if dist <= self.threshold:
            rospy.loginfo(f"[到达航点 {self.current_waypoint_index+1}/{len(self.waypoints)}]")
            
            # Move to the next waypoint
            self.current_waypoint_index += 1
            
            # Wait a bit before going to the next waypoint
            if self.current_waypoint_index < len(self.waypoints):
                rospy.loginfo(f"等待 {self.waiting_time} 秒后前往下一个航点...")
                rospy.sleep(self.waiting_time)
                self.navigate_to_current_waypoint()
            else:
                rospy.loginfo("[完成] 所有航点已完成!")
                self.reached_final = True
                # rospy.signal_shutdown("任务完成")


if __name__ == "__main__":
    rospy.init_node("nav_waypoints_player")

    # ===== 修改这里定义航点 =====
    # 每个航点为 (x, y, theta) 其中 theta 为角度制
    waypoints = [
        (-8.44, 8.89, 0),            # 航点1
        (-5.24, -0.20, 117.68),      # 航点2
        (1.12, -0.41, -53.2),        # 航点3
        (0.0, 0.0, 0.0),             # 航点4 - 回到原点
    ]
    
    threshold = 0.5  # 到达目标点的距离阈值（米）

    node = NavWaypointPlayer(waypoints, threshold)
    rospy.spin()