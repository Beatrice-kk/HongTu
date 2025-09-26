#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
import sys
import math

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

class CmdVelController:
    def __init__(self, network_interface):
        # 初始化 Unitree SDK
        rospy.loginfo("Initializing Unitree LocoClient...")
        ChannelFactoryInitialize(0, network_interface)

        self.sport_client = LocoClient()
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()

        # 位置跟踪相关变量
        self.current_pose = None  # 当前机器人位置
        self.target_pose = None   # 目标位置
        self.has_target = False   # 是否有目标位置
        
        # 偏差阈值设置
        self.distance_threshold = 0.4  # 距离阈值 0.3米
        self.angle_threshold = math.radians(30)  # 角度阈值 20度
        
        # 订阅 /cmd_vel
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        rospy.loginfo("Subscribed to /cmd_vel")
        
        # 订阅当前机器人位置
        rospy.Subscriber("/robot_pose", PoseStamped, self.pose_callback)
        rospy.loginfo("Subscribed to /robot_pose")
        
        # 订阅目标位置
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        rospy.loginfo("Subscribed to /move_base_simple/goal")

    def pose_callback(self, msg: PoseStamped):
        """接收当前机器人位置"""
        self.current_pose = msg
        rospy.logdebug(f"Current pose: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
    
    def goal_callback(self, msg: PoseStamped):
        """接收目标位置"""
        self.target_pose = msg
        self.has_target = True
        rospy.loginfo(f"New goal received: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
    
    def calculate_distance(self, pose1, pose2):
        """计算两点之间的距离"""
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        return math.sqrt(dx*dx + dy*dy)
    
    def calculate_angle_difference(self, pose1, pose2):
        """计算两个位置之间的角度差"""
        # 计算从pose1到pose2的角度
        dx = pose2.pose.position.x - pose1.pose.position.x
        dy = pose2.pose.position.y - pose1.pose.position.y
        target_angle = math.atan2(dy, dx)
        
        # 获取当前机器人的朝向角度
        current_quat = pose1.pose.orientation
        current_yaw = math.atan2(2.0 * (current_quat.w * current_quat.z + current_quat.x * current_quat.y),
                                1.0 - 2.0 * (current_quat.y * current_quat.y + current_quat.z * current_quat.z))
        
        # 计算角度差
        angle_diff = target_angle - current_yaw
        # 将角度差标准化到[-π, π]范围
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
            
        return abs(angle_diff)
    
    def is_at_target(self):
        """检查是否到达目标位置"""
        if not self.has_target or self.current_pose is None or self.target_pose is None:
            return False
            
        distance = self.calculate_distance(self.current_pose, self.target_pose)
        angle_diff = self.calculate_angle_difference(self.current_pose, self.target_pose)
        
        rospy.logdebug(f"Distance to target: {distance:.2f}m, Angle diff: {math.degrees(angle_diff):.1f}°")
        
        return distance <= self.distance_threshold and angle_diff <= self.angle_threshold

    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x      # 前后移动
        vy = msg.linear.y      # 横向移动
        wz = msg.angular.z     # 旋转

        # 检查是否到达目标位置
        if self.is_at_target():
            rospy.loginfo("Robot has reached target position. Stopping.")
            try:
                # 发送停止指令
                self.sport_client.Move(0.0, 0.0, 0.0)
            except Exception as e:
                rospy.logerr(f"Failed to send stop command: {e}")
            return

        # 如果 cmd_vel 全 0，则停止机器人
        if vx == 0.0 and vy == 0.0 and wz == 0.0:
            rospy.loginfo("Received cmd_vel is all zeros. Robot will stop.")
            try:
                self.sport_client.Move(0.0, 0.0, 0.0)
            except Exception as e:
                rospy.logerr(f"Failed to send stop command: {e}")
            return

        rospy.loginfo(f"Received cmd_vel: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}")
        try:
            # 发送运动指令
            self.sport_client.Move(vx, vy, wz)
        except Exception as e:
            rospy.logerr(f"Failed to send Move command: {e}")


if __name__ == "__main__":
   #  if len(sys.argv) < 2:
   #      print(f"Usage: rosrun your_package cmd_vel_control.py networkInterface")
   #      sys.exit(-1)

   #  network_interface = sys.argv[1]

      #   看具体网卡的名称 
    network_interface = "eth0"  


    rospy.init_node("unitree_cmd_vel_controller", anonymous=False)
    rospy.logwarn("Make sure the robot is in a safe environment before sending cmd_vel commands!")

    controller = CmdVelController(network_interface)

    rospy.spin()
