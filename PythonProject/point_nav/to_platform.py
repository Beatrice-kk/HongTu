#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import os
import threading
import tf.transformations as tft
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback
from geometry_msgs.msg import Twist
from std_msgs.msg import Header


class MultiWaypointNavigator:
    def __init__(self, waypoints, threshold_dist=0.5, threshold_yaw=5.0):
        """
        Initialize the multi-waypoint navigator

        Args:
            waypoints: List of waypoints, each as [x, y, yaw_degrees, dwell_time_seconds]
            threshold_dist: Distance threshold in meters
            threshold_yaw: Yaw threshold in degrees
        """
        self.waypoints = waypoints
        self.threshold_dist = threshold_dist
        self.threshold_yaw_deg = threshold_yaw
        self.threshold_yaw_rad = math.radians(threshold_yaw)
        self.current_waypoint_index = 0
        self.reached = False
        self.waiting = False
        self.wait_timer = None

        # Publisher for navigation goals
        self.goal_pub = rospy.Publisher(
            "/move_base/goal", MoveBaseActionGoal, queue_size=1
        )

        # Publisher to stop the robot
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Subscriber for position feedback
        self.feedback_sub = rospy.Subscriber(
            "/move_base/feedback", MoveBaseActionFeedback, self.feedback_callback
        )

        rospy.sleep(1.0)  # Wait for connections to establish

        # Start navigation with the first waypoint
        if self.waypoints:
            self.navigate_to_next_waypoint()

    def navigate_to_next_waypoint(self):
        """Navigate to the next waypoint in the list"""
        if self.current_waypoint_index >= len(self.waypoints):
            rospy.loginfo("所有航点已完成! 程序将自动关闭...")
            # Shutdown the node after a short delay to ensure the message is printed
            rospy.Timer(
                rospy.Duration(1),
                lambda _: rospy.signal_shutdown("All waypoints completed"),
                oneshot=True,
            )
            return

        waypoint = self.waypoints[self.current_waypoint_index]
        target_x, target_y, target_yaw_deg, dwell_time = waypoint

        # Convert yaw from degrees to radians
        target_yaw_rad = math.radians(target_yaw_deg)

        self.target_x = target_x
        self.target_y = target_y
        self.target_yaw_rad = target_yaw_rad
        self.target_yaw_deg = target_yaw_deg
        self.dwell_time = dwell_time
        self.reached = False

        self.publish_goal()

    def publish_goal(self):
        """Publish the current waypoint as a navigation goal"""
        goal = MoveBaseActionGoal()
        goal.goal.target_pose.header.frame_id = "map"
        goal.goal.target_pose.header.stamp = rospy.Time.now()
        goal.goal.target_pose.pose.position.x = self.target_x
        goal.goal.target_pose.pose.position.y = self.target_y

        # Convert yaw to quaternion
        q = tft.quaternion_from_euler(0, 0, self.target_yaw_rad)
        goal.goal.target_pose.pose.orientation.x = q[0]
        goal.goal.target_pose.pose.orientation.y = q[1]
        goal.goal.target_pose.pose.orientation.z = q[2]
        goal.goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo(
            f"[导航目标 {self.current_waypoint_index+1}/{len(self.waypoints)}] "
            f"x={self.target_x}, y={self.target_y}, θ={self.target_yaw_deg}° "
            f"停留时间: {self.dwell_time}秒"
        )
        self.goal_pub.publish(goal)

    def stop_robot(self):
        """Force the robot to stop by publishing zero velocity"""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0
        stop_cmd.linear.y = 0
        stop_cmd.linear.z = 0
        stop_cmd.angular.x = 0
        stop_cmd.angular.y = 0
        stop_cmd.angular.z = 0

        # Publish multiple times to ensure the robot stops
        for _ in range(3):
            self.cmd_vel_pub.publish(stop_cmd)
            rospy.sleep(0.1)

    def get_yaw_from_quaternion(self, orientation):
        """Extract yaw angle from quaternion"""
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = tft.euler_from_quaternion(quaternion)
        return yaw

    def calculate_yaw_diff(self, current_yaw, target_yaw):
        """Calculate the smallest angle difference between two yaw angles"""
        diff = abs(current_yaw - target_yaw) % (2 * math.pi)
        if diff > math.pi:
            diff = 2 * math.pi - diff
        return diff

    def feedback_callback(self, msg):
        """Handle position feedback from move_base"""
        if self.reached or self.waiting:
            return

        current_pose = msg.feedback.base_position.pose
        current_x = current_pose.position.x
        current_y = current_pose.position.y
        current_yaw = self.get_yaw_from_quaternion(current_pose.orientation)

        # Calculate position distance
        dx = current_x - self.target_x
        dy = current_y - self.target_y
        dist = math.hypot(dx, dy)

        # Calculate yaw difference
        yaw_diff = self.calculate_yaw_diff(current_yaw, self.target_yaw_rad)
        yaw_diff_deg = math.degrees(yaw_diff)

        rospy.loginfo_throttle(
            2,
            f"[当前位置] ({current_x:.2f}, {current_y:.2f}, {math.degrees(current_yaw):.1f}°) -> "
            f"距离目标: {dist:.2f}m, 角度差: {yaw_diff_deg:.1f}°",
        )

        # Check if both position and orientation thresholds are met
        if dist <= self.threshold_dist and yaw_diff <= self.threshold_yaw_rad:
            self.reached = True
            self.waiting = True

            rospy.loginfo(
                f"[到达目标 {self.current_waypoint_index+1}/{len(self.waypoints)}] "
                f"停止机器人，等待 {self.dwell_time} 秒..."
            )

            # Stop the robot
            self.stop_robot()

            # Wait for the specified dwell time before moving to the next waypoint
            self.wait_timer = threading.Timer(
                self.dwell_time, self.proceed_to_next_waypoint
            )
            self.wait_timer.start()

    def proceed_to_next_waypoint(self):
        """Move to the next waypoint after dwell time has elapsed"""
        self.waiting = False
        self.current_waypoint_index += 1
        rospy.loginfo(f"等待结束，前往下一个航点...")
        self.navigate_to_next_waypoint()


if __name__ == "__main__":
    rospy.init_node("multi_waypoint_navigator")

    # Define waypoints: [x, y, yaw_degrees, dwell_time_seconds]
    waypoints = [
        [2.9, 4.0, 180, 10.0],  # First waypoint, 10 seconds dwell
        #   [-5.0, 7.0, 90.0, 5.0],  # Second waypoint, 5 seconds dwell
        #   [-2.0, 3.0, 180.0, 8.0],  # Third waypoint, 8 seconds dwell
        [-0.6, 0.0, 0, 1.0],  # Fourth waypoint, 3 seconds dwell
    ]

    # Distance and yaw thresholds
    threshold_dist = 0.5  # meters
    threshold_yaw = 10.0  # degrees

    # Start the navigator
    navigator = MultiWaypointNavigator(waypoints, threshold_dist, threshold_yaw)
    rospy.spin()
