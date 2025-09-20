#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import os
import threading
import time
import tf.transformations as tft
import argparse
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.srv import GetPlan, GetPlanRequest
from nav_msgs.msg import Odometry

class NavWaypointPlayer:
    def __init__(self, backstage_pos, stage_entry_pos, dance_type, dance_choreography, threshold=0.8):
        """
        使用航点信息初始化导航器。
        """
        # 检查舞蹈类型是否存在于编排中
        if dance_type not in dance_choreography:
            rospy.logerr(f"舞蹈类型 '{dance_type}' 未定义编排，将使用默认编排")
            self.dance_sequence = list(dance_choreography.values())[0]
        else:
            self.dance_sequence = dance_choreography[dance_type]
        
        # 提取舞蹈点位和等待时间
        dance_waypoints = [pos for pos, _ in self.dance_sequence]
        self.wait_times = [0]
        self.wait_times.append(self.dance_sequence[0][1])
        for i in range(1, len(self.dance_sequence)):
            self.wait_times.append(self.dance_sequence[i][1])
        self.wait_times.append(0)
        
        # 构建完整的航点序列
        self.original_waypoints = [backstage_pos] + dance_waypoints + [backstage_pos]
        
      #   self.original_waypoints = [backstage_pos, stage_entry_pos] + dance_waypoints + [stage_entry_pos, backstage_pos]
        self.waypoints = self.original_waypoints.copy()  # 可能会被分段修改的工作副本
        self.threshold = threshold
        self.current_waypoint_index = 0
        self.reached_final = False

        # 添加主航点索引跟踪 - 记录当前处理的是哪个原始航点
        self.current_main_waypoint_index = 0

        # 分段导航相关
        self.sub_waypoint_index = None  # None 表示未分段
        
        # 当前位置缓存（避免等待消息超时）
        self.current_position = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.position_updated = False

        # 新增：记录舞蹈服务是否已经被调用
        self.dance_service_called = False
        
        # 确定在哪些点位需要等待
        # 修改：不再用于决定是否跳舞，只用于决定是否等待
        self.wait_at_waypoint = [False]
        self.wait_at_waypoint.append(True)
        self.wait_at_waypoint.extend([True] * len(dance_waypoints))
        self.wait_at_waypoint.append(False)
        
        self.dance_type = dance_type
        self.dance_in_progress = False
        self.navigation_active = False

        # 规划相关
        self.make_plan = None
        try:
            rospy.wait_for_service('/move_base/make_plan', timeout=2.0)
            self.make_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
            rospy.loginfo("连接 make_plan 成功")
        except Exception as e:
            rospy.logwarn(f"make_plan 服务不可用: {e}，将使用基于距离的退避策略")

        # 进度/卡滞检测
        self._last_progress_check_time = rospy.Time.now()
        self._last_progress_dist = float('inf')
        self._stuck_timeout = rospy.Duration(3.0)
        self._progress_eps = 0.1
        self._stuck_counter = 0
        self._max_stuck_attempts = 3
        
        # 定时器 - 强制继续进度
        self.progress_timer = None
        self.progress_check_interval = 5.0
        
        # 发布导航目标
        self.goal_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=1)
        self.feedback_sub = rospy.Subscriber("/move_base/feedback", MoveBaseActionFeedback, self.feedback_callback)
        self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        
        # 订阅里程计数据 - 作为位置备份
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        # 直接控制速度的发布器
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
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
        
        # 添加每个航点的最大停留时间
        self.max_time_per_waypoint = [70.0] * len(self.waypoints)
        self.waypoint_start_time = None
        
        # 启动进度检查定时器
        self.start_progress_timer()
        
        rospy.sleep(1.0)
        rospy.loginfo(f"开始表演，舞蹈类型: {self.dance_type}")
        self.navigate_to_current_waypoint()

    def start_progress_timer(self):
        """启动定期检查进度的定时器"""
        if self.progress_timer:
            self.progress_timer.shutdown()
        self.progress_timer = rospy.Timer(rospy.Duration(self.progress_check_interval), 
                                         self._check_progress_callback)

    def _check_progress_callback(self, event):
        """定期检查导航进度，确保不会永远卡住"""
        if self.reached_final or self.dance_in_progress:
            return
            
        if not self.navigation_active:
            return
            
        # 检查从开始导航到现在的时间
        if self.waypoint_start_time:
            elapsed_time = (rospy.Time.now() - self.waypoint_start_time).to_sec()
            max_time = self.max_time_per_waypoint[self.current_waypoint_index]
            
            if elapsed_time > max_time:
                rospy.logwarn(f"[超时检测] 航点 {self.current_waypoint_index+1}/{len(self.waypoints)} (主航点 {self.current_main_waypoint_index+1}) 导航时间 {elapsed_time:.1f}秒 > {max_time}秒，执行恢复操作")
                self._handle_timeout()

    def _build_move_base_goal(self, x, y, theta_deg):
        """
        构造MoveBaseActionGoal消息用于发布到move_base/goal
        """
        goal = MoveBaseActionGoal()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.goal_id.stamp = rospy.Time.now()
        goal.goal_id.id = ""
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

    def _build_ps(self, x, y, theta_deg):
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = rospy.Time.now()
        ps.pose.position.x = x
        ps.pose.position.y = y
        q = tft.quaternion_from_euler(0, 0, math.radians(theta_deg))
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]
        return ps

    def _split_path_linear(self, start_xy, end_xytheta, num_segments=5):
        """增加分段数量，让每一步更小"""
        sx, sy = start_xy
        ex, ey, eth = end_xytheta
        points = []
        for i in range(1, num_segments):
            ratio = i / num_segments
            x = sx + (ex - sx) * ratio
            y = sy + (ey - sy) * ratio
            points.append((x, y, eth))
        points.append((ex, ey, eth))
        return points

    def _try_find_reachable_nearby(self, target_xytheta, current_xy):
        """尝试找到附近可达点，如果失败直接返回分段路径"""
        # 简化处理 - 不再尝试使用make_plan，直接分段
        split_points = self._split_path_linear(current_xy, target_xytheta, num_segments=5)
        return split_points

    def odom_callback(self, msg):
        """从里程计获取当前位置作为备份"""
        self.current_position["x"] = msg.pose.pose.position.x
        self.current_position["y"] = msg.pose.pose.position.y
        
        # 提取角度
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = tft.euler_from_quaternion(quaternion)
        self.current_position["theta"] = math.degrees(euler[2])
        
        self.position_updated = True

    def _emergency_move_toward(self, target_x, target_y, current_x, current_y, duration=3.0):
        """紧急情况下直接发送速度命令向目标方向移动"""
        dx = target_x - current_x
        dy = target_y - current_y
        dist = math.hypot(dx, dy)
        
        if dist < 0.4:  # 已经非常接近
            return
            
        # 归一化方向向量
        if dist > 0:
            dx /= dist
            dy /= dist
        else:
            dx, dy = 1.0, 0.0  # 默认前进
        
        # 创建速度命令 - 使用较低的速度以确保安全
        cmd = Twist()
        cmd.linear.x = 0.1 * dx  # 降低线速度
        cmd.linear.y = 0.1 * dy
        cmd.angular.z = 0.0
        
        rospy.logwarn(f"执行紧急移动 {duration} 秒，方向: ({dx:.2f}, {dy:.2f})")
        
        # 发送一段时间的速度命令
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10Hz
        while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()
        
        # 停止
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def navigate_to_current_waypoint(self):
        """导航到当前索引的航点"""
        if self.current_waypoint_index >= len(self.waypoints):
            rospy.loginfo("[完成] 所有航点已完成!")
            self.reached_final = True
            return

        # 取消当前目标（如果有）
        cancel_msg = GoalID()
        self.cancel_pub.publish(cancel_msg)
        rospy.sleep(0.5)  # 短暂等待取消生效

        wp = self.waypoints[self.current_waypoint_index]
        # 分段点列支持
        if isinstance(wp, list):
            sub_index = self.sub_waypoint_index if self.sub_waypoint_index is not None else 0
            x, y, theta = wp[sub_index]
            location_desc = f"[分段航点 {sub_index+1}/{len(wp)} 属于主航点 {self.current_main_waypoint_index+1}/{len(self.original_waypoints)}]"
        else:
            x, y, theta = wp
            self.current_main_waypoint_index = self.current_waypoint_index  # 更新主航点索引
            location_desc = ""
            if self.current_waypoint_index == 0:
                location_desc = "[后台起点]"
            elif self.current_waypoint_index == 1:
                location_desc = "[舞台入口]"
            elif self.current_waypoint_index == len(self.original_waypoints) - 2:
                location_desc = "[舞台退出]"
            elif self.current_waypoint_index == len(self.original_waypoints) - 1:
                location_desc = "[返回后台]"
            else:
                location_desc = f"[舞蹈点位 {self.current_waypoint_index-1}]"

        wait_time = self.wait_times[self.current_waypoint_index] if self.current_waypoint_index < len(self.wait_times) else 0
        rospy.loginfo(f"[导航目标 {self.current_waypoint_index+1}/{len(self.waypoints)}] {location_desc} x={x}, y={y}, θ={theta} (等待时间: {wait_time}秒)")
        
        # 发布MoveBaseActionGoal
        self.goal_pub.publish(self._build_move_base_goal(x, y, theta))
        self._last_progress_check_time = rospy.Time.now()
        self._last_progress_dist = float('inf')
        self.waypoint_start_time = rospy.Time.now()
        self.navigation_active = True
        
    def _handle_timeout(self):
        """处理超时情况，确保机器人继续前进"""
        if self.dance_in_progress or self.reached_final:
            return
            
        rospy.logwarn(f"[超时处理] 航点 {self.current_waypoint_index+1} (主航点 {self.current_main_waypoint_index+1}) 导航超时，强制继续")
        
        # 使用缓存的位置信息，避免等待消息
        if not self.position_updated:
            rospy.logwarn("没有位置信息可用，无法执行紧急移动")
            # 即使没有位置信息也必须继续
            self._force_proceed_to_next()
            return
            
        current_x = self.current_position["x"]
        current_y = self.current_position["y"]
        
        wp = self.waypoints[self.current_waypoint_index]
        if isinstance(wp, list):
            sub_index = self.sub_waypoint_index if self.sub_waypoint_index is not None else 0
            target_x, target_y, _ = wp[sub_index]
            
            # 尝试紧急移动
            self._emergency_move_toward(target_x, target_y, current_x, current_y, duration=2.0)
            
            # 更新分段索引或整体航点索引
            sub_index += 1
            if sub_index < len(wp):
                self.sub_waypoint_index = sub_index
                nx, ny, nth = wp[sub_index]
                rospy.loginfo(f"[强制分段] 前往下一个分段点: x={nx}, y={ny}, θ={nth}")
                self.goal_pub.publish(self._build_move_base_goal(nx, ny, nth))
                self._last_progress_check_time = rospy.Time.now()
                self.waypoint_start_time = rospy.Time.now()
                return
            else:
                # 完成所有分段，前进到下一个主航点
                self._force_proceed_to_next()
        else:
            target_x, target_y, _ = wp
            # 尝试紧急移动
            self._emergency_move_toward(target_x, target_y, current_x, current_y, duration=2.0)
            # 强制前进到下一个航点
            self._force_proceed_to_next()
    
    def _force_proceed_to_next(self):
        """强制前进到下一个航点，无论当前位置如何"""
        self.sub_waypoint_index = None  # 重置分段
        current_main_index = self.current_main_waypoint_index
        self.current_waypoint_index += 1  # 前进到下一个航点
        
        # 如果当前是一个非分段的主航点，更新主航点索引
        if self.current_waypoint_index < len(self.waypoints) and not isinstance(self.waypoints[self.current_waypoint_index], list):
            self.current_main_waypoint_index = self.current_waypoint_index
        
        # 判断是否需要在当前点位执行舞蹈
        if self.current_waypoint_index > 0 and self.current_waypoint_index <= len(self.wait_at_waypoint):
            should_wait = self.wait_at_waypoint[self.current_waypoint_index - 1]
            dance_thread = threading.Thread(target=self._execute_dance_and_continue, args=(should_wait,))
            dance_thread.daemon = True
            dance_thread.start()
        else:
            # 直接继续到下一个航点
            self.dance_in_progress = False
            if self.current_waypoint_index < len(self.waypoints):
                self.navigate_to_current_waypoint()
            else:
                rospy.loginfo("[完成] 所有航点已完成!")
                self.reached_final = True

    def perform_dance(self):
        """
        调用舞蹈服务，只在第一次调用时真正执行，无需关注执行结果
        """
        if self.dance_service_called:
            rospy.loginfo("舞蹈服务已经被调用过，不再重复调用")
            return
            
        if self.play_dance_service is None:
            rospy.logwarn("舞蹈服务不可用，跳过舞蹈")
            return
            
        try:
            dance_direction = self.dance_type
            rospy.loginfo(f"开始执行舞蹈: {dance_direction}，本次表演中将只调用一次舞蹈服务")
            self.dance_direction_pub.publish(String(dance_direction))
            rospy.sleep(0.5)
            # 调用服务但不关心结果
            self.play_dance_service()
            # 标记舞蹈服务已调用
            self.dance_service_called = True
        except rospy.ServiceException as e:
            rospy.logerr(f"舞蹈服务调用失败: {e}")
        # 无论服务成功与否，都不影响后续流程

    def _execute_dance_and_continue(self, should_wait):
        """
        执行舞蹈动作并继续导航
        使用主航点索引而不是普通航点索引来判断是否调用舞蹈服务
        """
        try:
            # 修改：根据主航点索引而不是当前航点索引判断是否调用舞蹈服务
            # 第三个主航点(索引2)是第一个舞蹈点位
            if self.current_main_waypoint_index == 3 and not self.dance_service_called:
                rospy.loginfo(f"/* 到达第一个舞蹈点位(主航点 {self.current_main_waypoint_index+1})，开始执行舞蹈动作... */")
                self.perform_dance()
                rospy.loginfo("/* 舞蹈指令已发送，将继续执行剩余航点 */")
            
            # 获取等待时间
            wait_time = 0
            if self.current_waypoint_index > 0 and self.current_waypoint_index <= len(self.wait_times):
                wait_time = self.wait_times[self.current_waypoint_index - 1]
            
            # 只有主航点序列中的点才需要等待（跳过第一个和最后一个主航点）
            if should_wait and wait_time > 0 and self.current_main_waypoint_index > 1:
                rospy.loginfo(f"等待 {wait_time} 秒后前往下一个航点...")
                # 使用time.sleep而不是rospy.sleep，以便能够响应Ctrl+C
                start_time = time.time()
                while time.time() - start_time < wait_time and not rospy.is_shutdown():
                    time.sleep(0.1)
            else:
                rospy.loginfo("立即前往下一个航点...")
            
            if self.current_waypoint_index < len(self.waypoints):
                self._stuck_counter = 0  # 重置卡滞计数
                self.navigate_to_current_waypoint()
            else:
                rospy.loginfo("[完成] 所有航点已完成!")
                self.reached_final = True
        except Exception as e:
            rospy.logerr(f"舞蹈执行线程错误: {e}")
        finally:
            self.dance_in_progress = False
            
        
    def feedback_callback(self, msg):
        """处理导航反馈，检测卡滞和到达情况"""
        if self.reached_final or self.dance_in_progress:
            return

        # 更新当前位置缓存
        self.current_position["x"] = msg.feedback.base_position.pose.position.x
        self.current_position["y"] = msg.feedback.base_position.pose.position.y
        
        orientation = msg.feedback.base_position.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = tft.euler_from_quaternion(quaternion)
        self.current_position["theta"] = math.degrees(euler[2])
        
        self.position_updated = True

        # 支持分段点列
        wp = self.waypoints[self.current_waypoint_index]
        if isinstance(wp, list):
            sub_waypoints = wp
            sub_index = self.sub_waypoint_index if self.sub_waypoint_index is not None else 0
            x, y, _ = sub_waypoints[sub_index]
        else:
            x, y, _ = wp
            sub_waypoints = None

        current_pose = msg.feedback.base_position.pose
        dx = current_pose.position.x - x
        dy = current_pose.position.y - y
        dist = math.hypot(dx, dy)

        rospy.loginfo_throttle(2, f"[当前位置] ({current_pose.position.x:.2f}, {current_pose.position.y:.2f}) -> 距离目标 {dist:.2f} m (主航点 {self.current_main_waypoint_index+1})")

        # 检查是否超过了最大停留时间
        if self.waypoint_start_time and (rospy.Time.now() - self.waypoint_start_time).to_sec() > self.max_time_per_waypoint[self.current_waypoint_index]:
            rospy.logwarn(f"[超时] 航点 {self.current_waypoint_index+1} (主航点 {self.current_main_waypoint_index+1}) 导航超时，强制继续")
            self._handle_timeout()
            return

        now = rospy.Time.now()
        # 卡滞处理 - 更快地检测
        if now - self._last_progress_check_time > self._stuck_timeout and not self.reached_final and not self.dance_in_progress:
            progressed = False
            if self._last_progress_dist != float('inf'):
                progressed = (self._last_progress_dist - dist) > self._progress_eps
            self._last_progress_check_time = now
            self._last_progress_dist = dist

            if not progressed:
                self._stuck_counter += 1
                rospy.logwarn(f"[卡滞] 检测到卡滞 #{self._stuck_counter}，尝试恢复...")
                
                # 如果多次卡滞，尝试更激进的恢复策略
                if self._stuck_counter >= self._max_stuck_attempts:
                    rospy.logwarn(f"[严重卡滞] 连续卡滞 {self._stuck_counter} 次，强制移动")
                    # 直接向目标方向移动
                    self._emergency_move_toward(x, y, current_pose.position.x, current_pose.position.y, duration=2.0)
                    self._stuck_counter = 0  # 重置卡滞计数
                    
                    # 强制进入下一段或下一个航点
                    if isinstance(wp, list):
                        sub_index = self.sub_waypoint_index if self.sub_waypoint_index is not None else 0
                        sub_index += 1
                        if sub_index < len(wp):
                            self.sub_waypoint_index = sub_index
                            nx, ny, nth = wp[sub_index]
                            rospy.loginfo(f"[强制分段] 前往下一个分段点: x={nx}, y={ny}, θ={nth}")
                            self.goal_pub.publish(self._build_move_base_goal(nx, ny, nth))
                            self._last_progress_check_time = rospy.Time.now()
                            self.waypoint_start_time = rospy.Time.now()
                        else:
                            self._force_proceed_to_next()
                    else:
                        self._force_proceed_to_next()
                    return
                
                # 直接使用分段导航 - 不再尝试复杂规划
                cur_xy = (current_pose.position.x, current_pose.position.y)
                target = wp if not sub_waypoints else sub_waypoints[sub_index]
                
                # 获取分段路径
                split_points = self._split_path_linear(cur_xy, target, num_segments=5)
                self.waypoints[self.current_waypoint_index] = split_points
                self.sub_waypoint_index = 0
                nx, ny, nth = split_points[0]
                rospy.loginfo(f"[分段导航] 使用分段目标: x={nx:.2f}, y={ny:.2f}, θ={nth:.1f}")
                
                # 发布第一个分段点
                self.goal_pub.publish(self._build_move_base_goal(nx, ny, nth))
                self._last_progress_check_time = rospy.Time.now()
                self._last_progress_dist = float('inf')
                self.waypoint_start_time = rospy.Time.now()
            else:
                # 有进展，重置卡滞计数
                self._stuck_counter = 0

        # 更新最短距离记录
        if dist < self._last_progress_dist:
            self._last_progress_dist = dist

        # 检查是否到达目标
        if dist <= self.threshold:
            self.navigation_active = False
            self.dance_in_progress = True
            
            if sub_waypoints:
                sub_index = self.sub_waypoint_index if self.sub_waypoint_index is not None else 0
                sub_index += 1
                if sub_index < len(sub_waypoints):
                    # 前往下一个分段点
                    self.sub_waypoint_index = sub_index
                    nx, ny, nth = sub_waypoints[sub_index]
                    rospy.loginfo(f"[分段到达] 前往下一个分段点: x={nx}, y={ny}, θ={nth}")
                    self.goal_pub.publish(self._build_move_base_goal(nx, ny, nth))
                    self.dance_in_progress = False
                    self.navigation_active = True
                    self._stuck_counter = 0  # 重置卡滞计数
                    self.waypoint_start_time = rospy.Time.now()  # 更新开始时间
                    return
                else:
                    # 完成所有分段，前进到下一个主航点
                    self.sub_waypoint_index = None
                    self.current_waypoint_index += 1
                    # 如果不是分段点，更新主航点索引
                    if self.current_waypoint_index < len(self.waypoints) and not isinstance(self.waypoints[self.current_waypoint_index], list):
                        self.current_main_waypoint_index = self.current_waypoint_index
            else:
                # 直接前进到下一个航点
                self.current_waypoint_index += 1
                # 更新主航点索引
                if self.current_waypoint_index < len(self.waypoints) and not isinstance(self.waypoints[self.current_waypoint_index], list):
                    self.current_main_waypoint_index = self.current_waypoint_index

            # 判断是否需要在当前点位等待
            idx = self.current_waypoint_index - 1 if not sub_waypoints else self.current_waypoint_index
            if idx < len(self.wait_at_waypoint):
                should_wait = self.wait_at_waypoint[idx]
                dance_thread = threading.Thread(target=self._execute_dance_and_continue, args=(should_wait,))
                dance_thread.daemon = True
                dance_thread.start()
            else:
                # 超出范围，直接继续
                self.dance_in_progress = False
                if self.current_waypoint_index < len(self.waypoints):
                    self.navigate_to_current_waypoint()
                else:
                    rospy.loginfo("[完成] 所有航点已完成!")
                    self.reached_final = True

if __name__ == "__main__":
    rospy.init_node("nav_waypoints_player")
    parser = argparse.ArgumentParser(description='导航舞蹈表演控制器')
    parser.add_argument('--dance', type=str, default='A', 
                      choices=['A', 'B', 'Up', 'Down', 'Left', 'Right', 'X', 'Y'],
                      help='指定要执行的舞蹈类型')
    args, unknown = parser.parse_known_args()
    backstage_pos =(0.0,0,0)
    stage_entry_pos = (-1.94,0.80,115)

    dance_choreography = {
        'A': [
            ((-2.85, 3.0, 151), 60.0),
            ((-2.8, 4.1, 100), 50.0),
            ((-3.7, 2.7, 160), 70.0),
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
    threshold = 0.8  # 到达阈值
    node = NavWaypointPlayer(
        backstage_pos=backstage_pos,
        stage_entry_pos=stage_entry_pos,
        dance_type=args.dance,
        dance_choreography=dance_choreography,
        threshold=threshold
    )
    rospy.loginfo(f"表演开始，使用舞蹈类型: {args.dance}")
    rospy.spin()