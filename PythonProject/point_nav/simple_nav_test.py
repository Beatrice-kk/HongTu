#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import threading
import time
import tf.transformations as tft
import argparse
import sys
import os
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback
from actionlib_msgs.msg import GoalID
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist

# 添加SDK路径以导入G1ActionPlayer
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../unitree_sdk2_python/example/g1/high_level/"))
from g1_action_player import G1ActionPlayer

class SimpleNavTest:
    def __init__(self, dance_type, dance_choreography):
        """
        简化的导航舞蹈测试类
        """
        # 获取舞蹈序列
        if dance_type not in dance_choreography:
            rospy.logerr(f"舞蹈类型 '{dance_type}' 未定义，使用默认")
            self.dance_sequence = list(dance_choreography.values())[0]
        else:
            self.dance_sequence = dance_choreography[dance_type]

        # 提取舞蹈位置和等待时间
        self.dance_waypoints = [pos for pos, _ in self.dance_sequence]
        self.wait_times = [wait for _, wait in self.dance_sequence]
        
        # 添加后台点
        backstage_pos = (-0.6, 0, 0)
        if self.dance_waypoints and self.dance_waypoints[-1] != backstage_pos:
            self.waypoints = self.dance_waypoints + [backstage_pos]
        else:
            self.waypoints = self.dance_waypoints

        rospy.loginfo(f"总路径点数量: {len(self.waypoints)}")
        rospy.loginfo(f"路径点序列: {self.waypoints}")

        # 基本参数
        self.threshold = 0.5  # 距离阈值
        self.angle_threshold = 15.0  # 角度阈值
        self.current_waypoint_index = 0
        self.current_position = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.dance_service_called = False
        self.dance_type = dance_type

        # Publishers and subscribers
        self.goal_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=1)
        self.feedback_sub = rospy.Subscriber("/move_base/feedback", MoveBaseActionFeedback, self.feedback_callback)
        self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self.dance_direction_pub = rospy.Publisher("dance_direction", String, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # 舞蹈服务
        try:
            rospy.wait_for_service("play_dance", timeout=5.0)
            self.play_dance_service = rospy.ServiceProxy("play_dance", Trigger)
            rospy.loginfo("舞蹈服务已连接")
        except rospy.ROSException:
            rospy.logwarn("舞蹈服务不可用")
            self.play_dance_service = None

        rospy.loginfo(f"开始测试，舞蹈类型: {self.dance_type}")
        self.navigate_to_current_waypoint()

    def _build_move_base_goal(self, x, y, theta_deg):
        """构建导航目标"""
        goal = MoveBaseActionGoal()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.goal_id.stamp = rospy.Time.now()
        goal.goal_id.id = f"nav_test_{rospy.Time.now().to_nsec()}"
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

    def navigate_to_current_waypoint(self):
        """导航到当前路径点"""
        if self.current_waypoint_index >= len(self.waypoints):
            rospy.loginfo("所有路径点已完成!")
            rospy.signal_shutdown("任务完成")
            return

        # 取消之前的目标
        cancel_msg = GoalID()
        self.cancel_pub.publish(cancel_msg)
        rospy.sleep(0.1)

        # 获取当前路径点
        x, y, theta = self.waypoints[self.current_waypoint_index]
        
        # 发布导航目标
        goal_msg = self._build_move_base_goal(x, y, theta)
        rospy.loginfo(f"导航到路径点 {self.current_waypoint_index+1}/{len(self.waypoints)}: x={x}, y={y}, θ={theta}")
        self.goal_pub.publish(goal_msg)

    def perform_dance(self):
        """执行舞蹈"""
        if self.dance_service_called:
            rospy.loginfo("舞蹈已执行，跳过")
            return

        if self.play_dance_service is None:
            rospy.logwarn("舞蹈服务不可用，跳过舞蹈")
            self.move_to_next_waypoint()
            return

        try:
            # 停止机器人
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            rospy.sleep(0.5)
            
            rospy.loginfo(f"开始舞蹈: {self.dance_type}")
            self.dance_direction_pub.publish(String(self.dance_type))
            rospy.sleep(0.5)

            # 调用舞蹈服务
            response = self.play_dance_service()
            if response.success:
                rospy.loginfo("舞蹈完成")
            else:
                rospy.logwarn(f"舞蹈失败: {response.message}")
            
            self.dance_service_called = True
            self.move_to_next_waypoint()

        except Exception as e:
            rospy.logerr(f"舞蹈执行失败: {e}")
            self.move_to_next_waypoint()

    def move_to_next_waypoint(self):
        """移动到下一个路径点"""
        # 等待当前点的等待时间
        if self.current_waypoint_index < len(self.wait_times):
            wait_time = self.wait_times[self.current_waypoint_index]
            if wait_time > 0:
                rospy.loginfo(f"等待 {wait_time} 秒...")
                rospy.sleep(wait_time)

        # 移动到下一个点
        self.current_waypoint_index += 1
        if self.current_waypoint_index < len(self.waypoints):
            rospy.loginfo(f"移动到下一个路径点...")
            self.navigate_to_current_waypoint()
        else:
            rospy.loginfo("所有路径点已完成!")
            rospy.signal_shutdown("任务完成")

    def feedback_callback(self, msg):
        """处理导航反馈"""
        if self.dance_service_called and self.current_waypoint_index > 0:
            return  # 舞蹈已执行，跳过后续检查

        # 更新当前位置
        current_pose = msg.feedback.base_position.pose
        self.current_position["x"] = current_pose.position.x
        self.current_position["y"] = current_pose.position.y

        orientation = current_pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = tft.euler_from_quaternion(quaternion)
        self.current_position["theta"] = math.degrees(euler[2])

        # 获取当前目标点
        x, y, target_yaw = self.waypoints[self.current_waypoint_index]

        # 计算距离和角度差
        dx = current_pose.position.x - x
        dy = current_pose.position.y - y
        dist = math.hypot(dx, dy)

        current_yaw = self.current_position["theta"]
        d_yaw = abs(current_yaw - target_yaw)
        if d_yaw > 180:
            d_yaw = 360 - d_yaw

        # 检查是否到达目标点
        if dist <= self.threshold and d_yaw <= self.angle_threshold:
            rospy.loginfo(f"到达路径点 {self.current_waypoint_index+1} (距离:{dist:.2f}m, 角度差:{d_yaw:.2f}度)")
            
            # 如果是第一个点且还没执行舞蹈，执行舞蹈
            if self.current_waypoint_index == 0 and not self.dance_service_called:
                rospy.loginfo("到达第一个舞蹈点，开始舞蹈")
                self.perform_dance()
            else:
                # 其他点直接移动到下一个
                self.move_to_next_waypoint()


if __name__ == "__main__":
    rospy.init_node("simple_nav_test")
    parser = argparse.ArgumentParser(description="简化导航舞蹈测试")
    parser.add_argument("--dance", type=str, default="A", 
                       choices=["A", "B", "X", "Y", "Up", "Down"],
                       help="指定舞蹈类型")
    args = parser.parse_known_args()[0]

    # 舞蹈编排
    dance_choreography = {
        "A": [
            ((-3.6, 3.4, 170), 30.0),
            ((-3.4, 3.4, 180), 40.0),
            ((-3.2, 3.4, -170), 20.0),
            ((-0.6, 0, 0), 0),
        ],
        "B": [
            ((-2.1, 3.4, 170), 3.0),
            ((-2.3, 3.4, 180), 4.0),
            ((-2.6, 3.4, -170), 2.0),
            ((-1.5, 3.4, 180), 2.5),
            ((-0.6, 0, 0), 0),
        ],

        "X": [
            ((4.18, 1.15, -159), 3.0),
            ((-3.0, 3.0, 135), 4.0),
            ((3.0, -3.0, -45), 3.5),
            ((-3.0, -3.0, -135), 2.5),
            ((3.0, 3.0, 45), 3.0),
            ((-0.6, 0, 0), 0),
        ],
        "Y": [
            ((-3.6, 3.4, 170), 30.0),
            ((-3.4, 3.4, 180), 40.0),
            ((-3.2, 3.4, -170), 20.0),
            ((-0.6, 0, 0), 0),
        ],
        "Q": [
            ((-3.6, 3.4, 170), 30.0),
        ],
        "H": [
            ((-0.6, 0, 0), 30.0),
        ],
    }
    # 检查是否是Up或Down参数
    if args.dance in ["Up", "Down"]:
        rospy.loginfo(f"检测到{args.dance}参数，直接调用TTS和动作播放")
        try:
            action_player = G1ActionPlayer()
            if args.dance == "Up":
                tts_text = action_player.tts_presets.get('B', "各位朋友，大家好。在江南水乡沙家浜，曾镌刻下一段军民同心、共抗敌寇的红色记忆。这里有指导员郭建光的壮志凌云，有阿庆嫂的机智沉着，有沙奶奶的慈爱坚毅，也有与敌人周旋的惊心动魄。接下来，让我们循着京剧《沙家浜》的经典旋律，一同穿越烽火岁月，重温那段充满斗争智慧与深厚情谊的历史！")
                action_dir = "start_b"
            else:  # Down
                tts_text = action_player.tts_presets.get('C', "各位朋友，经典的唱腔余韵悠长，烽火里的故事依旧动人。我们刚刚一同重温了郭建光的壮志、沙奶奶的坚韧，也深深记住了阿庆嫂垒起七星灶的过人智慧，更读懂了那份跨越岁月的军民鱼水情。本场沙家浜京剧选段演出到此圆满结束，感谢您的驻足与陪伴，我们下次再会！")
                action_dir = "start_x"
            
            action_player._play_tts_with_action(tts_text, action_dir, 0)
            rospy.loginfo(f"TTS和动作播放完成: {args.dance}")
        except Exception as e:
            rospy.logerr(f"调用_play_tts_with_action失败: {e}")
        finally:
            rospy.signal_shutdown("任务完成")
    else:
        # 正常的导航舞蹈逻辑
        node = SimpleNavTest(
            dance_type=args.dance,
            dance_choreography=dance_choreography,
        )
        rospy.spin()
