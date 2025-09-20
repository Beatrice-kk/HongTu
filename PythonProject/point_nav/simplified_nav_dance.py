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

class SimpleNavWaypointPlayer:
    def __init__(self, backstage_pos, stage_entry_pos, dance_type, dance_choreography, threshold=0.8):
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
      #   self.waypoints = [backstage_pos, stage_entry_pos] + dance_waypoints + [backstage_pos]
        self.waypoints = dance_waypoints + [backstage_pos]
      
        self.threshold = threshold
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
        
        # Publishers and subscribers
        self.goal_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=1)
        self.feedback_sub = rospy.Subscriber("/move_base/feedback", MoveBaseActionFeedback, self.feedback_callback)
        self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
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
            rospy.logwarn(f"[STUCK] Robot has moved only {dist_moved:.3f}m in {time_diff:.1f}s, trying to recover...")
            
            # Check if we've sent the goal recently
            if self.last_goal_send_time and (current_time - self.last_goal_send_time).to_sec() < 10.0:
                self.goal_send_retries += 1
                if self.goal_send_retries >= self.max_goal_retries:
                    rospy.logwarn("[NAVIGATION FAILURE] Multiple attempts to set goal failed. Forcing move to next waypoint.")
                    self.current_waypoint_index += 1
                    self.goal_send_retries = 0
                    if self.current_waypoint_index < len(self.waypoints):
                        self.navigate_to_current_waypoint()
                    else:
                        rospy.loginfo("[COMPLETE] All waypoints completed!")
                        self.reached_final = True
                    return
            
            # Try to re-send the goal after forcefully clearing move_base
            rospy.loginfo("Resetting navigation system and resending goal...")
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
        
        rospy.loginfo("Robot stopped completely")

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

    def odom_callback(self, msg):
        """Get current position from odometry as backup"""
        self.current_position["x"] = msg.pose.pose.position.x
        self.current_position["y"] = msg.pose.pose.position.y
        
        # Extract angle
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = tft.euler_from_quaternion(quaternion)
        self.current_position["theta"] = math.degrees(euler[2])

    def navigate_to_current_waypoint(self, is_retry=False):
        """Navigate to the current waypoint"""
        if self.current_waypoint_index >= len(self.waypoints):
            rospy.loginfo("[COMPLETE] All waypoints completed!")
            self.reached_final = True
            return

        # Reset for new navigation attempt
        if not is_retry:
            self.goal_send_retries = 0
            
        # First ensure any previous goals are canceled
        self.reset_navigation()

        # Get current waypoint
        x, y, theta = self.waypoints[self.current_waypoint_index]
        
        # Determine location description
        if self.current_waypoint_index == 0:
            location_desc = "[Backstage Start]"
        elif self.current_waypoint_index == 1:
            location_desc = "[Stage Entry]"
        elif self.current_waypoint_index == len(self.waypoints) - 1:
            location_desc = "[Return to Backstage]"
        else:
            location_desc = f"[Dance Position {self.current_waypoint_index-1}]"

        wait_time = self.wait_times[self.current_waypoint_index-2] if self.current_waypoint_index >= 2 and self.current_waypoint_index < len(self.wait_times) + 2 else 0
        prefix = "[RETRY] " if is_retry else ""
        rospy.loginfo(f"{prefix}[Navigation Target {self.current_waypoint_index+1}/{len(self.waypoints)}] {location_desc} x={x}, y={y}, θ={theta} (Wait time: {wait_time}s)")
        
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
            rospy.loginfo("Dance service has already been called, not calling again")
            return
            
        if self.play_dance_service is None:
            rospy.logwarn("Dance service not available, skipping dance")
            return
            
        try:
            dance_direction = self.dance_type
            rospy.loginfo(f"Starting dance: {dance_direction}, will only call dance service once in this performance")
            self.dance_direction_pub.publish(String(dance_direction))
            rospy.sleep(0.5)
            
            # Call service in a non-blocking way
            self.play_dance_service()
            
            # Mark dance service as called
            self.dance_service_called = True
            rospy.loginfo("Dance service called, will proceed with navigation after waiting")
        except Exception as e:
            rospy.logerr(f"Dance service call failed: {e}")
            # Even if the dance fails, we should continue

    def execute_dance_and_wait(self):
        """Execute dance, wait for specified time, then continue"""
        try:
            # First, ensure the robot is completely stopped
            self.stop_robot()
            
            # Check if we should start dancing (at first dance position)
            if self.current_waypoint_index == 1 and not self.dance_service_called:
                rospy.loginfo("/* Reached first dance position, starting dance... */")
                self.perform_dance()
                rospy.loginfo("/* Dance command sent, will continue with remaining waypoints after waiting */")
            
            # Get wait time for current position
            wait_time = 0
            if self.current_waypoint_index >= 2 and self.current_waypoint_index < len(self.wait_times) + 2:
                wait_time = self.wait_times[self.current_waypoint_index - 2]
            
            if wait_time > 0:
                rospy.loginfo(f"Waiting {wait_time} seconds before going to next waypoint...")
                # Keep publishing stop commands periodically during wait
                start_time = time.time()
                while time.time() - start_time < wait_time and not rospy.is_shutdown():
                    if int(time.time() - start_time) % 2 == 0:  # Every 2 seconds
                        self.stop_robot()
                    time.sleep(0.1)
            else:
                rospy.loginfo("Immediately proceeding to next waypoint...")
            
            # Move to next waypoint
            self.current_waypoint_index += 1
            self.dance_in_progress = False
            
            if self.current_waypoint_index < len(self.waypoints):
                self.navigate_to_current_waypoint()
            else:
                rospy.loginfo("[COMPLETE] All waypoints completed!")
                self.reached_final = True
                
        except Exception as e:
            rospy.logerr(f"Dance execution error: {e}")
            # Ensure we continue even if there's an error
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
        x, y, _ = self.waypoints[self.current_waypoint_index]

        # Calculate distance to target
        dx = current_pose.position.x - x
        dy = current_pose.position.y - y
        dist = math.hypot(dx, dy)

        rospy.loginfo_throttle(2, f"[Current Position] ({current_pose.position.x:.2f}, {current_pose.position.y:.2f}) -> Distance to target: {dist:.2f} m")

        # Check if we've reached the waypoint
        if dist <= self.threshold and not self.dance_in_progress:
            rospy.loginfo(f"Reached waypoint {self.current_waypoint_index+1}")
            self.navigation_active = False
            self.dance_in_progress = True
            
            # Execute dance and wait in a separate thread
            dance_thread = threading.Thread(target=self.execute_dance_and_wait)
            dance_thread.daemon = True
            dance_thread.start()

if __name__ == "__main__":
    rospy.init_node("simple_nav_waypoints_player")
    parser = argparse.ArgumentParser(description='Navigation Dance Performance Controller')
    parser.add_argument('--dance', type=str, default='A', 
                      choices=['A', 'B', 'Up', 'Down', 'Left', 'Right', 'X', 'Y'],
                      help='Specify dance type to execute')
    args, unknown = parser.parse_known_args()
    
    backstage_pos = (-1.65, 0, 0)
    stage_entry_pos = (-1.94, 0.80, 115)

    dance_choreography = {
        'A': [
            ((-2.4, 3.54, 136), 20.0),
            ((-3.0, 4.2, 157), 30.0),
            ((-3.7, 2.5, 152), 20.0),
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
    
    threshold = 0.8  # Arrival threshold
    node = SimpleNavWaypointPlayer(
        backstage_pos=backstage_pos,
        stage_entry_pos=stage_entry_pos,
        dance_type=args.dance,
        dance_choreography=dance_choreography,
        threshold=threshold
    )
    
    rospy.loginfo(f"Performance started, using dance type: {args.dance}")
    rospy.spin()