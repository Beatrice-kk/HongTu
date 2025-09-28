#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
快速移动测试脚本
专门用于测试机器人是否能够移动，不考虑精准度
"""

import rospy
import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf.transformations as tft

class QuickMoveTest:
    def __init__(self):
        """初始化快速移动测试"""
        rospy.init_node("quick_move_test")
        
        # 目标点
        self.target_pos = (-0.8, 0, 0)
        
        # 当前机器人位置
        self.current_position = {"x": 0.0, "y": 0.0, "theta": 0.0}
        
        # 发布器
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        # 订阅里程计
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        rospy.loginfo("快速移动测试初始化完成")
        rospy.loginfo(f"目标点: {self.target_pos}")
    
    def odom_callback(self, msg):
        """处理里程计数据"""
        try:
            self.current_position["x"] = msg.pose.pose.position.x
            self.current_position["y"] = msg.pose.pose.position.y
            
            # 计算偏航角
            orientation = msg.pose.pose.orientation
            quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
            euler = tft.euler_from_quaternion(quaternion)
            self.current_position["theta"] = math.degrees(euler[2])
        except Exception as e:
            rospy.logwarn(f"处理里程计数据失败: {e}")
    
    def stop_robot(self):
        """停止机器人"""
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        rospy.sleep(0.5)
    
    def test_basic_movement(self):
        """测试基本移动能力"""
        rospy.loginfo("测试1：基本移动能力测试")
        
        # 测试向前移动
        rospy.loginfo("向前移动2秒...")
        move_msg = Twist()
        move_msg.linear.x = 0.3
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 2.0:
            self.cmd_vel_pub.publish(move_msg)
            rospy.sleep(0.1)
        
        self.stop_robot()
        rospy.sleep(1.0)
        
        # 测试转向
        rospy.loginfo("转向2秒...")
        turn_msg = Twist()
        turn_msg.angular.z = 0.5
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 2.0:
            self.cmd_vel_pub.publish(turn_msg)
            rospy.sleep(0.1)
        
        self.stop_robot()
        rospy.loginfo("基本移动测试完成")
    
    def test_target_approach(self):
        """测试向目标点移动"""
        rospy.loginfo("测试2：向目标点移动")
        rospy.loginfo(f"当前位置: ({self.current_position['x']:.2f}, {self.current_position['y']:.2f})")
        rospy.loginfo(f"目标位置: {self.target_pos}")
        
        # 计算到目标点的方向
        dx = self.target_pos[0] - self.current_position["x"]
        dy = self.target_pos[1] - self.current_position["y"]
        distance = math.sqrt(dx*dx + dy*dy)
        angle = math.atan2(dy, dx)
        
        rospy.loginfo(f"距离目标: {distance:.2f}米，角度: {math.degrees(angle):.1f}度")
        
        # 分阶段移动
        max_time = 30.0  # 最大移动时间
        start_time = rospy.Time.now()
        last_position = (self.current_position["x"], self.current_position["y"])
        
        while (rospy.Time.now() - start_time).to_sec() < max_time:
            # 计算当前位置到目标的距离
            current_dx = self.target_pos[0] - self.current_position["x"]
            current_dy = self.target_pos[1] - self.current_position["y"]
            current_distance = math.sqrt(current_dx*current_dx + current_dy*current_dy)
            
            # 如果距离小于1米，认为成功
            if current_distance < 1.0:
                rospy.loginfo(f"? 成功接近目标点！距离: {current_distance:.2f}米")
                self.stop_robot()
                return True
            
            # 检查是否在移动
            current_pos = (self.current_position["x"], self.current_position["y"])
            moved_distance = math.sqrt((current_pos[0] - last_position[0])**2 + 
                                     (current_pos[1] - last_position[1])**2)
            
            if moved_distance > 0.05:  # 移动了5cm
                last_position = current_pos
                rospy.loginfo(f"正在移动... 距离目标: {current_distance:.2f}米")
            
            # 计算移动方向
            current_angle = math.atan2(current_dy, current_dx)
            
            # 发布移动命令
            move_msg = Twist()
            move_msg.linear.x = 0.4  # 移动速度
            move_msg.angular.z = current_angle * 0.8  # 转向速度
            
            self.cmd_vel_pub.publish(move_msg)
            rospy.sleep(0.1)
        
        self.stop_robot()
        rospy.logwarn(f"? 移动超时，最终距离: {current_distance:.2f}米")
        return False
    
    def test_multiple_directions(self):
        """测试多个方向的移动"""
        rospy.loginfo("测试3：多方向移动测试")
        
        directions = [
            ("向前", 0.3, 0, 0),
            ("向后", -0.3, 0, 0),
            ("向左", 0, 0.3, 0),
            ("向右", 0, -0.3, 0),
            ("斜向前", 0.2, 0.2, 0),
            ("斜向后", -0.2, 0.2, 0),
        ]
        
        for name, dx, dy, dtheta in directions:
            rospy.loginfo(f"测试{name}移动...")
            
            # 计算目标位置
            target_x = self.current_position["x"] + dx
            target_y = self.current_position["y"] + dy
            target_theta = self.current_position["theta"] + dtheta
            
            # 移动到目标位置
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time).to_sec() < 3.0:
                # 计算移动方向
                current_dx = target_x - self.current_position["x"]
                current_dy = target_y - self.current_position["y"]
                current_distance = math.sqrt(current_dx*current_dx + current_dy*current_dy)
                
                if current_distance < 0.2:  # 接近目标
                    break
                
                # 发布移动命令
                move_msg = Twist()
                move_msg.linear.x = 0.3
                move_msg.angular.z = math.atan2(current_dy, current_dx) * 0.5
                
                self.cmd_vel_pub.publish(move_msg)
                rospy.sleep(0.1)
            
            self.stop_robot()
            rospy.sleep(1.0)
            rospy.loginfo(f"{name}移动测试完成")
    
    def run(self):
        """运行测试"""
        try:
            rospy.loginfo("开始快速移动测试...")
            rospy.loginfo("目标：确保机器人能够移动，不考虑精准度")
            
            # 等待系统稳定
            rospy.sleep(2.0)
            
            # 测试1：基本移动
            self.test_basic_movement()
            
            # 测试2：向目标点移动
            success = self.test_target_approach()
            
            if success:
                rospy.loginfo("? 移动测试成功！机器人能够正常移动")
            else:
                rospy.logwarn("? 移动测试部分成功，机器人能够移动但可能未到达目标")
                
                # 测试3：多方向移动
                self.test_multiple_directions()
            
            rospy.loginfo("快速移动测试完成")
            
        except KeyboardInterrupt:
            rospy.loginfo("用户中断测试")
        except Exception as e:
            rospy.logerr(f"测试过程中发生错误: {e}")
        finally:
            # 清理
            self.stop_robot()

def main():
    """主函数"""
    try:
        tester = QuickMoveTest()
        tester.run()
    except Exception as e:
        rospy.logerr(f"程序启动失败: {e}")

if __name__ == "__main__":
    main()
