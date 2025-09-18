#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionFeedback


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
        self.waiting_time = 30.0  # Seconds to wait between waypoints

        # 发布导航目标 (PoseStamped)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        # 订阅 /move_base/feedback 获取当前位姿
        self.feedback_sub = rospy.Subscriber("/move_base/feedback", MoveBaseActionFeedback, self.feedback_callback)

        rospy.sleep(1.0)  # 等待topic连接
        self.navigate_to_current_waypoint()

    def navigate_to_current_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints):
            rospy.loginfo("[完成] 所有航点已完成!")
            self.reached_final = True
            return

        x, y, theta = self.waypoints[self.current_waypoint_index]

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        q = tft.quaternion_from_euler(0, 0, math.radians(theta))  # Convert degrees to radians
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

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


if __name__ == "__main__":
    rospy.init_node("nav_waypoints_player_simple")

    # ===== 修改这里定义航点 =====
    waypoints = [
        (13.44, -0.38, -132),        # 航点1
        (10.121, 19.70, 138),         # 航点2
        (0.570, 1.213, 67),       # 航点3 - 回到原点
    ]

    threshold = 0.5  # 到达目标点的距离阈值（米）

    node = NavWaypointPlayer(waypoints, threshold)
    rospy.spin()
