#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import os
import threading
import tf.transformations as tft
import argparse
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
# from playsound import playsound


class NavWaypointPlayer:
    def __init__(self, backstage_pos, stage_entry_pos, dance_type, dance_choreography, threshold=0.5):
        """
        使用航点信息初始化导航器。
        
        参数:
            backstage_pos: 后台位置，(x, y, theta)元组
            stage_entry_pos: 舞台入口位置，(x, y, theta)元组
            dance_type: 要执行的舞蹈类型，如'A', 'B', 'Up'等
            dance_choreography: 舞蹈编排，字典格式，键为舞蹈类型，值为该舞蹈的点位序列
            threshold: 认为到达航点的距离阈值(米)
        """
        # 检查舞蹈类型是否存在于编排中
        if dance_type not in dance_choreography:
            rospy.logerr(f"舞蹈类型 '{dance_type}' 未定义编排，将使用默认编排")
            # 使用默认的舞蹈编排(第一个舞蹈的编排)
            self.dance_sequence = list(dance_choreography.values())[0]
        else:
            # 使用指定舞蹈类型的编排
            self.dance_sequence = dance_choreography[dance_type]
        
        # 提取舞蹈点位和等待时间
        dance_waypoints = [pos for pos, _ in self.dance_sequence]
        self.wait_times = [0]  # 后台起点等待时间为0
        self.wait_times.append(self.dance_sequence[0][1])  # 舞台入口的等待时间
        
        # 添加所有舞蹈点位的等待时间
        for i in range(1, len(self.dance_sequence)):
            self.wait_times.append(self.dance_sequence[i][1])
        
        self.wait_times.append(0)  # 返回后台后无需等待
        
        # 构建完整的航点序列
        self.waypoints = [backstage_pos, stage_entry_pos] + dance_waypoints + [backstage_pos]
        self.threshold = threshold
        self.current_waypoint_index = 0
        self.reached_final = False
        
        # 确定在哪些点位执行舞蹈 - 只在舞台入口和舞蹈点位执行
        self.dance_at_waypoint = [False]  # 后台不跳舞
        self.dance_at_waypoint.append(True)  # 舞台入口跳舞
        self.dance_at_waypoint.extend([True] * len(dance_waypoints))  # 舞蹈点位跳舞
        self.dance_at_waypoint.append(False)  # 返回后台不跳舞
        
        # 使用指定的舞蹈类型
        self.dance_type = dance_type
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
        rospy.loginfo(f"开始表演，舞蹈类型: {self.dance_type}")
        self.navigate_to_current_waypoint()

    def navigate_to_current_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints):
            rospy.loginfo("[完成] 所有航点已完成!")
            self.reached_final = True
            # rospy.signal_shutdown("任务完成")
            return

        x, y, theta = self.waypoints[self.current_waypoint_index]
        
        # 为当前位置添加描述
        location_desc = ""
        if self.current_waypoint_index == 0:
            location_desc = "[后台起点]"
        elif self.current_waypoint_index == 1:
            location_desc = "[舞台入口]"
        elif self.current_waypoint_index == len(self.waypoints) - 1:
            location_desc = "[返回后台]"
        else:
            location_desc = f"[舞蹈点位 {self.current_waypoint_index-1}]"
        
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

        wait_time = self.wait_times[self.current_waypoint_index]
        rospy.loginfo(f"[导航目标 {self.current_waypoint_index+1}/{len(self.waypoints)}] {location_desc} x={x}, y={y}, θ={theta} (等待时间: {wait_time}秒)")
        self.goal_pub.publish(goal)

    def perform_dance(self):
        """调用舞蹈服务执行舞蹈动作"""
        if self.play_dance_service is None:
            rospy.logwarn("舞蹈服务不可用，跳过舞蹈")
            return True
        
        try:
            # 使用指定的舞蹈类型
            dance_direction = self.dance_type
            
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
            # 构建当前位置描述
            location_desc = ""
            if self.current_waypoint_index == 0:
                location_desc = "后台起点"
            elif self.current_waypoint_index == 1:
                location_desc = "舞台入口"
            elif self.current_waypoint_index == len(self.waypoints) - 1:
                location_desc = "返回后台"
            else:
                location_desc = f"舞蹈点位 {self.current_waypoint_index-1}"
                
            rospy.loginfo(f"[到达航点 {self.current_waypoint_index+1}/{len(self.waypoints)}] {location_desc}")
            
            # 判断是否需要在当前点位执行舞蹈
            should_dance = self.dance_at_waypoint[self.current_waypoint_index]
            
            # 在此航点执行舞蹈（如果需要）
            self.dance_in_progress = True
            
            # 创建一个线程来执行舞蹈，这样不会阻塞反馈处理
            dance_thread = threading.Thread(target=self._execute_dance_and_continue, args=(should_dance,))
            dance_thread.daemon = True
            dance_thread.start()

    def _execute_dance_and_continue(self, should_dance):
        """在单独的线程中执行舞蹈，然后继续导航"""
        try:
            # 如果需要在此点位执行舞蹈
            if should_dance:
                # 执行舞蹈动作
                rospy.loginfo("/* 开始执行指定舞蹈动作... */")
                dance_success = self.perform_dance()
                rospy.loginfo("/* 舞蹈动作执行完毕 */")
            
            # 获取当前点位的等待时间
            wait_time = self.wait_times[self.current_waypoint_index]
            
            # 前往下一个航点
            self.current_waypoint_index += 1
            
            # 等待指定时间后继续
            if self.current_waypoint_index < len(self.waypoints):
                if wait_time > 0:
                    rospy.loginfo(f"等待 {wait_time} 秒后前往下一个航点...")
                    rospy.sleep(wait_time)
                else:
                    rospy.loginfo("立即前往下一个航点...")
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
    
    # 添加命令行参数解析
    parser = argparse.ArgumentParser(description='导航舞蹈表演控制器')
    parser.add_argument('--dance', type=str, default='A', 
                      choices=['A', 'B', 'Up', 'Down', 'Left', 'Right', 'X', 'Y'],
                      help='指定要执行的舞蹈类型')
    args, unknown = parser.parse_known_args()
    
    # 定义固定位置
    backstage_pos = (0.18, -0.7, -179)  # 后台位置
    stage_entry_pos = (4.18, 1.15, -159)  # 舞台入口位置
    
    # 为每种舞蹈类型定义不同的点位和等待时间
    # 格式: 舞蹈类型 -> [((x, y, theta), wait_time), ...]
    dance_choreography = {
        # A型舞蹈的编排
        'A': [
            ((4.18, 1.15, -159), 3.0),             # 舞台入口，等待3秒
            ((-5.238, -0.204, 117.677), 5.0),      # 点位1，等待5秒
            ((1.122, -0.413, -53.2), 2.0),         # 点位2，等待2秒
        ],
        
        # B型舞蹈的编排
        'B': [
            ((4.18, 1.15, -159), 2.0),             # 舞台入口，等待2秒
            ((-3.5, 0.5, 90), 4.0),                # 点位1，等待4秒
            ((2.3, 1.7, -45), 3.0),                # 点位2，等待3秒
            ((-1.5, -1.2, 180), 2.5),              # 点位3，等待2.5秒
        ],
        
        # Up型舞蹈的编排
        'Up': [
            ((4.18, 1.15, -159), 2.5),             # 舞台入口，等待2.5秒
            ((3.0, 3.0, 0), 3.0),                  # 点位1，等待3秒
            ((-2.0, 2.0, 45), 4.0),                # 点位2，等待4秒
        ],
        
        # Down型舞蹈的编排
        'Down': [
            ((4.18, 1.15, -159), 1.5),             # 舞台入口，等待1.5秒
            ((0.0, 0.0, -90), 3.5),                # 点位1，等待3.5秒
            ((2.5, -2.0, 135), 2.0),               # 点位2，等待2秒
            ((-2.5, -1.0, -135), 4.5),             # 点位3，等待4.5秒
        ],
        
        # Left型舞蹈的编排
        'Left': [
            ((4.18, 1.15, -159), 2.0),             # 舞台入口，等待2秒
            ((-4.0, 1.0, 90), 3.0),                # 点位1，等待3秒
            ((-2.0, -1.0, -90), 2.5),              # 点位2，等待2.5秒
        ],
        
        # Right型舞蹈的编排
        'Right': [
            ((4.18, 1.15, -159), 2.0),             # 舞台入口，等待2秒
            ((3.0, -1.0, -90), 3.0),               # 点位1，等待3秒
            ((1.0, 2.0, 0), 2.5),                  # 点位2，等待2.5秒
        ],
        
        # X型舞蹈的编排
        'X': [
            ((4.18, 1.15, -159), 3.0),             # 舞台入口，等待3秒
            ((-3.0, 3.0, 135), 4.0),               # 点位1，等待4秒
            ((3.0, -3.0, -45), 3.5),               # 点位2，等待3.5秒
            ((-3.0, -3.0, -135), 2.5),             # 点位3，等待2.5秒
            ((3.0, 3.0, 45), 3.0),                 # 点位4，等待3秒
        ],
        
        # Y型舞蹈的编排
        'Y': [
            ((4.18, 1.15, -159), 2.5),             # 舞台入口，等待2.5秒
            ((0.0, 3.0, 90), 3.0),                 # 点位1，等待3秒
            ((-2.0, 0.0, 180), 4.0),               # 点位2，等待4秒
            ((2.0, 0.0, 0), 3.5),                  # 点位3，等待3.5秒
        ]
    }
    
    # 注意：这里的点位坐标是虚构的，请根据实际环境替换为真实坐标
    
    threshold = 0.5  # 到达目标点的距离阈值（米）

    node = NavWaypointPlayer(
        backstage_pos=backstage_pos,
        stage_entry_pos=stage_entry_pos,
        dance_type=args.dance,
        dance_choreography=dance_choreography,
        threshold=threshold
    )
    
    rospy.loginfo(f"表演开始，使用舞蹈类型: {args.dance}")
    rospy.spin()