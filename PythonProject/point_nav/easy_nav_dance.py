#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import threading
import time
import tf.transformations as tft
import argparse
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

class DirectWaypointNavigation:
    def __init__(self, backstage_pos, door_pos, dance_type, dance_choreography, 
                 pos_threshold=0.3, yaw_threshold=10, max_linear_speed=0.5, max_angular_speed=0.5):
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
        self.waypoints = dance_waypoints + [backstage_pos]
        
        rospy.loginfo(f"总路径点数量: {len(self.waypoints)}，舞蹈点: {len(dance_waypoints)}，最后还会回到: {backstage_pos}")
      
        # Navigation parameters
        self.pos_threshold = pos_threshold
        self.yaw_threshold = yaw_threshold
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        
        # Controller gains
        self.linear_gain = 0.5  # Proportional gain for linear velocity
        self.angular_gain = 1.0  # Proportional gain for angular velocity
        
        # Waypoint tracking
        self.current_waypoint_index = 0
        self.reached_final = False
        
        # Current position tracking
        self.current_position = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.position_initialized = False
        
        # Dance state tracking
        self.dance_service_called = False
        self.dance_type = dance_type
        self.dance_in_progress = False
        self.navigation_active = False
        
        # Obstacle detection
        self.obstacle_detected = False
        self.min_obstacle_distance = 0.5  # Minimum distance to consider an obstacle
        
        # Timing and control
        self.control_rate = rospy.Rate(10)  # 10Hz control loop
        self.last_control_time = rospy.Time.now()
        self.timeout_duration = 60.0  # Maximum time to reach waypoint (seconds)
        self.waypoint_start_time = None
        
        # Publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/slam_odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.dance_direction_pub = rospy.Publisher('dance_direction', String, queue_size=1)
        
        # For visualization (optional)
        self.target_pose_pub = rospy.Publisher('/target_pose', PoseStamped, queue_size=1)
        
        # Dance service client
        rospy.loginfo("Waiting for dance service...")
        try:
            rospy.wait_for_service('play_dance', timeout=5.0)
            self.play_dance_service = rospy.ServiceProxy('play_dance', Trigger)
            rospy.loginfo("Dance service connected")
        except rospy.ROSException:
            rospy.logwarn("Dance service not available, will not perform dance actions")
            self.play_dance_service = None
        
        # Navigation control thread
        self.nav_thread = None
        self.stop_nav_thread = False
        
        # Dance timer
        self.dance_timer = None
        
        rospy.sleep(1.0)
        rospy.loginfo(f"Starting performance, dance type: {self.dance_type}")
        
        # Wait for odometry to initialize
        self.wait_for_odom_init()
        
        # Start navigation control
        self.start_navigation()

    def wait_for_odom_init(self):
        """Wait for odometry to be initialized"""
        rospy.loginfo("Waiting for odometry data...")
        timeout = rospy.Time.now() + rospy.Duration(10.0)  # 10 second timeout
        
        while not self.position_initialized and rospy.Time.now() < timeout:
            rospy.sleep(0.1)
            
        if not self.position_initialized:
            rospy.logwarn("Timed out waiting for odometry data. Starting with zeroed position.")
        else:
            rospy.loginfo(f"Odometry initialized. Current position: ({self.current_position['x']:.2f}, {self.current_position['y']:.2f}, {self.current_position['theta']:.2f})")

    def odom_callback(self, msg):
        """Process odometry data to track robot position"""
        # Extract position
        self.current_position["x"] = msg.pose.pose.position.x
        self.current_position["y"] = msg.pose.pose.position.y
        
        # Extract orientation
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.current_position["theta"] = math.degrees(euler[2])
        
        # Mark as initialized once we've received valid data
        if not self.position_initialized:
            self.position_initialized = True

    def scan_callback(self, msg):
        """Process laser scan data for obstacle detection"""
        # Simple obstacle detection using a subset of ranges
        # Only consider ranges in front of the robot
        front_ranges = []
        
        # Check angles within ±30 degrees of the front
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        for i, r in enumerate(msg.ranges):
            angle = angle_min + i * angle_increment
            # Consider only front-facing angles (±45 degrees)
            if -math.pi/4 <= angle <= math.pi/4:
                if r > msg.range_min and r < msg.range_max:
                    front_ranges.append(r)
        
        # Check if any obstacles are too close
        if front_ranges and min(front_ranges) < self.min_obstacle_distance:
            if not self.obstacle_detected:
                rospy.logwarn(f"Obstacle detected at {min(front_ranges):.2f} meters")
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def start_navigation(self):
        """Start the navigation control thread"""
        if self.nav_thread is not None:
            self.stop_nav_thread = True
            self.nav_thread.join(timeout=1.0)
        
        self.stop_nav_thread = False
        self.nav_thread = threading.Thread(target=self.navigation_controller)
        self.nav_thread.daemon = True
        self.nav_thread.start()
        
        # Initialize the waypoint timer
        self.waypoint_start_time = rospy.Time.now()
        rospy.loginfo("Navigation controller started")

    def stop_navigation(self):
        """Stop the navigation control thread"""
        if self.nav_thread is not None:
            self.stop_nav_thread = True
            self.nav_thread.join(timeout=1.0)
            self.nav_thread = None
        
        # Make sure the robot stops
        self.stop_robot()
        rospy.loginfo("Navigation controller stopped")

    def navigation_controller(self):
        """Main navigation control loop"""
        while not rospy.is_shutdown() and not self.stop_nav_thread:
            if self.navigation_active and not self.dance_in_progress and not self.reached_final:
                # Get current waypoint
                if self.current_waypoint_index < len(self.waypoints):
                    # Check timeout
                    if (rospy.Time.now() - self.waypoint_start_time).to_sec() > self.timeout_duration:
                        rospy.logwarn(f"Timeout reaching waypoint {self.current_waypoint_index+1}. Moving to next waypoint.")
                        self.current_waypoint_index += 1
                        if self.current_waypoint_index < len(self.waypoints):
                            self.waypoint_start_time = rospy.Time.now()
                        else:
                            self.reached_final = True
                            self.stop_robot()
                            continue
                    
                    # Get target position
                    target_x, target_y, target_theta = self.waypoints[self.current_waypoint_index]
                    
                    # Calculate distance and angle to target
                    dx = target_x - self.current_position["x"]
                    dy = target_y - self.current_position["y"]
                    distance = math.hypot(dx, dy)
                    
                    # Calculate angle to target
                    target_angle = math.degrees(math.atan2(dy, dx))
                    
                    # Calculate angular difference (heading error)
                    heading_error = target_angle - self.current_position["theta"]
                    # Normalize to [-180, 180]
                    while heading_error > 180:
                        heading_error -= 360
                    while heading_error < -180:
                        heading_error += 360
                    
                    # Log progress
                    rospy.loginfo_throttle(2.0, f"Nav to waypoint {self.current_waypoint_index+1}: "
                                           f"dist={distance:.2f}m, heading_error={heading_error:.1f}°")
                    
                    # Publish target for visualization
                    self.publish_target_pose(target_x, target_y, target_theta)
                    
                    # Check if we've reached the waypoint
                    if distance < self.pos_threshold:
                        # We're close to the position, now check orientation
                        orientation_error = target_theta - self.current_position["theta"]
                        # Normalize to [-180, 180]
                        while orientation_error > 180:
                            orientation_error -= 360
                        while orientation_error < -180:
                            orientation_error += 360
                            
                        # Check if orientation is within threshold
                        if abs(orientation_error) < self.yaw_threshold:
                            rospy.loginfo(f"Reached waypoint {self.current_waypoint_index+1}!")
                            self.waypoint_reached()
                            continue
                        else:
                            # Just rotate to the target orientation
                            self.rotate_to_target(orientation_error)
                            self.control_rate.sleep()
                            continue
                    
                    # Handle obstacle avoidance
                    if self.obstacle_detected:
                        # Simple obstacle avoidance - stop and wait
                        self.stop_robot()
                        rospy.logwarn_throttle(1.0, "Obstacle detected. Waiting...")
                        self.control_rate.sleep()
                        continue
                    
                    # Move toward the target
                    # First turn to face the target if the heading error is large
                    if abs(heading_error) > 30:  # Only rotate if heading error is significant
                        self.rotate_to_target(heading_error)
                    else:
                        # Move toward the target with both linear and angular components
                        self.move_to_target(distance, heading_error)
                else:
                    # No more waypoints
                    self.reached_final = True
                    self.stop_robot()
            
            # Sleep to maintain control loop rate
            self.control_rate.sleep()

    def rotate_to_target(self, heading_error):
        """Rotate to face the target direction"""
        cmd = Twist()
        
        # Set angular velocity proportional to heading error
        angular_velocity = self.angular_gain * (heading_error / 180.0) * self.max_angular_speed
        
        # Limit angular velocity
        if abs(angular_velocity) > self.max_angular_speed:
            angular_velocity = math.copysign(self.max_angular_speed, angular_velocity)
        
        # Set minimum angular velocity to overcome friction
        min_angular_vel = 0.1
        if 0 < abs(angular_velocity) < min_angular_vel:
            angular_velocity = math.copysign(min_angular_vel, angular_velocity)
            
        cmd.angular.z = angular_velocity
        
        # Publish velocity command
        self.cmd_vel_pub.publish(cmd)

    def move_to_target(self, distance, heading_error):
        """Move toward the target with both linear and angular components"""
        cmd = Twist()
        
        # Set linear velocity proportional to distance, but cap it
        linear_velocity = self.linear_gain * distance
        if linear_velocity > self.max_linear_speed:
            linear_velocity = self.max_linear_speed
            
        # Set angular velocity proportional to heading error
        angular_velocity = self.angular_gain * (heading_error / 180.0) * self.max_angular_speed
        
        # Limit angular velocity
        if abs(angular_velocity) > self.max_angular_speed:
            angular_velocity = math.copysign(self.max_angular_speed, angular_velocity)
            
        # Reduce linear velocity when turning sharply
        turn_factor = 1.0 - (abs(heading_error) / 90.0) * 0.8
        if turn_factor < 0.2:
            turn_factor = 0.2
            
        cmd.linear.x = linear_velocity * turn_factor
        cmd.angular.z = angular_velocity
        
        # Publish velocity command
        self.cmd_vel_pub.publish(cmd)

    def waypoint_reached(self):
        """Handle logic when a waypoint is reached"""
        self.navigation_active = False
        self.stop_robot()
        
        # Check if it's a dance position
        if self.current_waypoint_index < len(self.wait_times):
            self.dance_in_progress = True
            rospy.loginfo(f"Starting dance at waypoint {self.current_waypoint_index+1}")
            
            # If this is the first waypoint, call the dance service
            if self.current_waypoint_index == 0 and not self.dance_service_called:
                self.perform_dance()
            
            # Wait for the specified time at this waypoint
            wait_time = self.wait_times[self.current_waypoint_index]
            if wait_time > 0:
                rospy.loginfo(f"Waiting at waypoint for {wait_time} seconds")
                if self.dance_timer:
                    self.dance_timer.shutdown()
                self.dance_timer = rospy.Timer(rospy.Duration(wait_time), 
                                               self.continue_to_next_waypoint, oneshot=True)
            else:
                # No wait time, continue immediately
                self.continue_to_next_waypoint()
        else:
            # No dance at this waypoint, continue to next
            self.continue_to_next_waypoint()

    def continue_to_next_waypoint(self, event=None):
        """Move to the next waypoint after dance/wait is complete"""
        self.dance_in_progress = False
        self.current_waypoint_index += 1
        
        if self.current_waypoint_index < len(self.waypoints):
            rospy.loginfo(f"Moving to waypoint {self.current_waypoint_index+1}/{len(self.waypoints)}")
            self.navigation_active = True
            self.waypoint_start_time = rospy.Time.now()
        else:
            rospy.loginfo("All waypoints completed!")
            self.reached_final = True
            self.stop_robot()

    def perform_dance(self):
        """Call the dance service"""
        if self.dance_service_called:
            rospy.loginfo("Dance service already called, not calling again")
            return
            
        if self.play_dance_service is None:
            rospy.logwarn("Dance service not available, skipping dance")
            return
            
        try:
            rospy.loginfo(f"Starting dance: {self.dance_type}")
            self.dance_direction_pub.publish(String(self.dance_type))
            rospy.sleep(0.5)
            
            # Call dance service in a non-blocking thread
            dance_thread = threading.Thread(target=self._call_dance_service)
            dance_thread.daemon = True
            dance_thread.start()
            
            # Mark dance service as called
            self.dance_service_called = True
            
        except Exception as e:
            rospy.logerr(f"Failed to call dance service: {e}")

    def _call_dance_service(self):
        """Helper to call dance service in a thread"""
        try:
            self.play_dance_service()
            rospy.loginfo("Dance service completed")
        except Exception as e:
            rospy.logerr(f"Dance service execution error: {e}")

    def stop_robot(self):
        """Send zero velocity command to stop the robot"""
        zero_vel = Twist()
        # Send multiple times to ensure it's received
        for _ in range(3):
            self.cmd_vel_pub.publish(zero_vel)
            rospy.sleep(0.05)
        rospy.loginfo("Robot stopped")

    def publish_target_pose(self, x, y, theta_deg):
        """Publish the current target pose for visualization"""
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert degrees to quaternion
        q = tft.quaternion_from_euler(0, 0, math.radians(theta_deg))
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        
        self.target_pose_pub.publish(pose)

def set_rotation(enable: bool):
    """Set robot rotation capability"""
    rospy.wait_for_service('/unitree_cmd_vel_controller/set_rotation_enabled')
    try:
        set_rotation_srv = rospy.ServiceProxy('/unitree_cmd_vel_controller/set_rotation_enabled', SetBool)
        resp = set_rotation_srv(enable)
        rospy.loginfo(f"Set rotation enabled={enable}: {resp.success}, {resp.message}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    # Enable rotation at startup
    rospy.init_node("direct_waypoint_navigation")
    
    parser = argparse.ArgumentParser(description='Direct Waypoint Navigation Controller')
    parser.add_argument('--dance', type=str, default='A', 
                       choices=['A', 'B', 'Up', 'Down', 'Left', 'Right', 'X', 'Y'],
                       help='Specify dance type to execute')
    parser.add_argument('--linear_speed', type=float, default=0.5,
                       help='Maximum linear speed (m/s)')
    parser.add_argument('--angular_speed', type=float, default=0.5,
                       help='Maximum angular speed (rad/s)')
    parser.add_argument('--pos_threshold', type=float, default=0.3,
                       help='Position arrival threshold (m)')
    parser.add_argument('--yaw_threshold', type=float, default=10.0,
                       help='Yaw arrival threshold (degrees)')
    
    args, unknown = parser.parse_known_args()
    
    # Set initial rotation capability
    set_rotation(True)
    
    backstage_pos = (-0.6, 0, 0)
    door_pos = (-1.6, 0, 0)  # Not used in this implementation but kept for compatibility

    dance_choreography = {
        'A': [
            ((-3.6, 3.4, 170), 20.0),
            ((-3.4, 3.4, 180), 30.0),
            ((-3.2, 3.4, -170), 20.0),
            ((-1.7,0.0,0.0), 0.0),
            
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
    
    # Toggle rotation based on waypoint
    try:
        node = DirectWaypointNavigation(
            backstage_pos=backstage_pos,
            door_pos=door_pos,
            dance_type=args.dance,
            dance_choreography=dance_choreography,
            pos_threshold=args.pos_threshold,
            yaw_threshold=args.yaw_threshold,
            max_linear_speed=args.linear_speed,
            max_angular_speed=args.angular_speed
        )
        
        # Toggle rotation off after reaching first waypoint
        if node.current_waypoint_index == 1:
            set_rotation(False)
            rospy.loginfo("Reached first dance point - disabling rotation")
        
        rospy.loginfo(f"Performance started, dance type: {args.dance}")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Clean shutdown
        if 'node' in locals():
            node.stop_navigation()
            node.stop_robot()