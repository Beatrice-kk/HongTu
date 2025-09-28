#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import sys
import signal
import time

# 使用readchar库读取单个字符
try:
    import readchar
except ImportError:
    print("请先安装readchar库: pip install readchar")
    sys.exit(1)

class KeyboardTeleop:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('keyboard_teleop', anonymous=True)
        
        # 创建发布者，发布到cmd_vel话题
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # 创建Twist消息对象
        self.twist = Twist()
        
        # 设置线性和角速度增量
        self.linear_speed = 0.2  # 单位: m/s
        self.angular_speed = 0.5  # 单位: rad/s
        
        # 停止标志
        self.running = True
        
        # 显示帮助信息
        self.print_instructions()
        
        # 设置信号处理器，用于捕获Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)
    
    def print_instructions(self):
        """打印操作指南"""
        print("\n=====================================================")
        print("                 键盘控制机器人")
        print("=====================================================")
        print("控制说明:")
        print("w/↑ - 前进")
        print("s/↓ - 后退")
        print("a/← - 左转")
        print("d/→ - 右转")
        print("空格 - 停止")
        print("q - 退出程序")
        print("=====================================================")
        print("当前设置:")
        print(f"线性速度: {self.linear_speed} m/s")
        print(f"角速度: {self.angular_speed} rad/s")
        print("=====================================================\n")
        print("正在等待按键输入...")
    
    def process_key(self, key):
        """处理按键输入"""
        if key in ['w', '\x1b[A']:  # w 或 上箭头
            self.twist.linear.x = self.linear_speed
            self.twist.angular.z = 0.0
            print("前进")
            return True
        elif key in ['s', '\x1b[B']:  # s 或 下箭头
            self.twist.linear.x = -self.linear_speed
            self.twist.angular.z = 0.0
            print("后退")
            return True
        elif key in ['a', '\x1b[D']:  # a 或 左箭头
            self.twist.linear.x = 0.0
            self.twist.angular.z = self.angular_speed
            print("左转")
            return True
        elif key in ['d', '\x1b[C']:  # d 或 右箭头
            self.twist.linear.x = 0.0
            self.twist.angular.z = -self.angular_speed
            print("右转")
            return True
        elif key == ' ':  # 空格
            self.stop_robot()
            print("停止")
            return True
        elif key == 'q':  # q键退出
            self.running = False
            print("退出程序")
            return False
        return True
    
    def stop_robot(self):
        """停止机器人"""
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
    
    def run(self):
        """主循环"""
        rate = rospy.Rate(10)  # 10Hz
        
        while self.running and not rospy.is_shutdown():
            # 读取按键
            key = readchar.readkey()
            
            # 处理按键
            if not self.process_key(key):
                break
            
            # 发布速度命令
            self.cmd_vel_pub.publish(self.twist)
            rate.sleep()
        
        # 确保退出前停止机器人
        self.stop_robot()
        self.cmd_vel_pub.publish(self.twist)
    
    def signal_handler(self, sig, frame):
        """处理Ctrl+C信号"""
        print("\n程序被用户中断")
        self.running = False
        self.stop_robot()
        self.cmd_vel_pub.publish(self.twist)  # 发布一次停止命令
        sys.exit(0)

if __name__ == "__main__":
    try:
        teleop = KeyboardTeleop()
        teleop.run()
    except rospy.ROSInterruptException:
        pass