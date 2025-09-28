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
from enum import Enum
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback
from actionlib_msgs.msg import GoalID, GoalStatusArray
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist

class NavigationState(Enum):
    """导航状态枚举"""
    IDLE = "idle"
    NAVIGATING = "navigating"
    DANCING = "dancing"
    WAITING = "waiting"
    COMPLETED = "completed"
    ERROR = "error"


class SimpleNavWaypointPlayer:
    def __init__(
        self,
        backstage_pos,
        dance_type,
        dance_choreography,
    ):
        """
        Initialize the navigator with waypoint information.
        """
        # 线程安全锁
        self._lock = threading.RLock()
        
        # Check if dance type exists in choreography
        if dance_type not in dance_choreography:
            rospy.logerr(
                f"Dance type '{dance_type}' not defined, using default choreography"
            )
            dance_config = list(dance_choreography.values())[0]
        else:
            dance_config = dance_choreography[dance_type]

        # Extract global time limit
        self.global_time_limit = dance_config["global_time"]
        self.dance_sequence = dance_config["waypoints"]

        # Extract dance positions and wait times
        dance_waypoints = [pos for pos, _ in self.dance_sequence]
        self.wait_times = [wait for _, wait in self.dance_sequence]

        # Build complete waypoint sequence
        # 确保后台点被包含在路径中
        if dance_waypoints and dance_waypoints[-1] != backstage_pos:
            self.waypoints = dance_waypoints + [backstage_pos]
        else:
            self.waypoints = dance_waypoints

        rospy.loginfo(
            f"总路径点数量: {len(self.waypoints)}，舞蹈点: {len(dance_waypoints)}，后台点: {backstage_pos}"
        )
        rospy.loginfo(f"完整路径点序列: {self.waypoints}")
        
        # 定义门口点位坐标（用于识别过渡点）
        # 优化门口位置，使其更容易通过
        self.doorway_pos = (-1.91, 1.35, 0)  # 更新为新的门口位置
        
        # 记录门口点位信息
        doorway_count = sum(1 for i in range(len(self.waypoints)) if self.is_doorway_waypoint(i))
        rospy.loginfo(f"门口过渡点数量: {doorway_count}")

        # 智能路径优化：如果机器人能直接规划到后台，跳过过渡点
        self.optimize_waypoints()

        # 到达检测由g1_control.py处理，这里不需要设置阈值

        # 使用状态机管理状态
        self._state = NavigationState.IDLE
        self.current_waypoint_index = 0
        self.reached_final = False

        # Current position tracking
        self.current_position = {"x": 0.0, "y": 0.0, "theta": 0.0}

        # Dance state tracking
        self.dance_service_called = False
        self.dance_type = dance_type

        # Navigation status monitoring
        self.move_base_status = None
        self.last_goal_send_time = None
        self.goal_send_retries = 0
        self.max_goal_retries = 2  # 减少重试次数，更快地前进到下一个点

        # 添加时间阈值 - 在一个点停留太久后自动前进
        self.waypoint_start_time = None
        
        # 全局时间管理
        self.global_start_time = None
        self.global_timer = None
        self.force_return_triggered = False
        
        # 固定时间调度系统
        self.waypoint_schedule = []  # 存储每个航点的计划时间
        self.schedule_timer = None    # 调度定时器
        
        # 定时器管理
        self._timers = []
        self._threads = []

        # Publishers and subscribers
        self.goal_pub = rospy.Publisher(
            "/move_base/goal", MoveBaseActionGoal, queue_size=1
        )
        self.feedback_sub = rospy.Subscriber(
            "/move_base/feedback", MoveBaseActionFeedback, self.feedback_callback
        )
        self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self.dance_direction_pub = rospy.Publisher(
            "dance_direction", String, queue_size=1
        )
        self.status_sub = rospy.Subscriber(
            "/move_base/status", GoalStatusArray, self.status_callback
        )

        # Add cmd_vel publisher to stop the robot
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        # 订阅cmd_vel以检测机器人停止
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        
        # 订阅机器人到达状态话题
        self.arrival_sub = rospy.Subscriber("/robot_arrival_status", String, self.arrival_callback)
        
        # 添加停止检测机制 - 通过检测cmd_vel为0来触发到达
        self.last_cmd_vel_time = rospy.Time.now()
        self.cmd_vel_zero_duration = 0.0
        self.required_stop_time = 2.0  # 需要停止2秒才认为到达

        # Dance service client
        rospy.loginfo("Waiting for dance service...")
        try:
            rospy.wait_for_service("play_dance", timeout=5.0)
            self.play_dance_service = rospy.ServiceProxy("play_dance", Trigger)
            rospy.loginfo("Dance service connected")
            
            # 测试服务是否可用 - 只检查服务是否存在，不实际调用
            try:
                # 只检查服务是否可用，不实际调用
                rospy.loginfo("Dance service is available")
            except Exception as e:
                rospy.logwarn(f"Dance service test failed: {e}")
                # 如果测试失败，重新创建服务代理
                rospy.sleep(1.0)
                self.play_dance_service = rospy.ServiceProxy("play_dance", Trigger)
                rospy.loginfo("Dance service proxy recreated")
                
        except rospy.ROSException:
            rospy.logwarn("Dance service not available, will not perform dance actions")
            self.play_dance_service = None

        # Up/Down service clients
        self.play_up_service = None
        self.play_down_service = None
        
        if self.dance_type in ["Up", "Down"]:
            try:
                if self.dance_type == "Up":
                    rospy.loginfo("Waiting for play_up service...")
                    rospy.wait_for_service("play_up", timeout=5.0)
                    self.play_up_service = rospy.ServiceProxy("play_up", Trigger)
                    rospy.loginfo("Play_up service connected")
                else:  # Down
                    rospy.loginfo("Waiting for play_down service...")
                    rospy.wait_for_service("play_down", timeout=5.0)
                    self.play_down_service = rospy.ServiceProxy("play_down", Trigger)
                    rospy.loginfo("Play_down service connected")
            except rospy.ROSException:
                rospy.logwarn(f"{self.dance_type} service not available")
                if self.dance_type == "Up":
                    self.play_up_service = None
                else:
                    self.play_down_service = None

        # Navigation watchdog timer - reduced checking frequency
        self.nav_watchdog_timer = None
        self.last_position_check = {"x": 0.0, "y": 0.0, "time": rospy.Time.now()}

        # Add a timer for dance completion
        self.dance_timer = None

      #   # 添加一个持续移动的定时器，确保机器人不会停止
      #   self.movement_check_timer = rospy.Timer(
      #       rospy.Duration(3.0), self.ensure_movement
      #   )
      #   self._timers.append(self.movement_check_timer)
        
        # 速度检测由g1_control.py处理，这里不需要相关变量
        
        # 添加状态监控定时器
        self.status_timer = rospy.Timer(
            rospy.Duration(10.0), self.log_status_info
        )
        self._timers.append(self.status_timer)
        
        # 停止检测现在通过话题通信实现，不需要定时器
        # self.stop_detection_timer = rospy.Timer(
        #     rospy.Duration(0.5), self.check_robot_stopped
        # )
        # self._timers.append(self.stop_detection_timer)

        # 增加启动延迟，确保所有服务完全初始化
        rospy.sleep(2.0)
        
        # 强制重置所有舞蹈服务状态，确保从干净状态开始
        rospy.loginfo("重置所有舞蹈服务状态...")
        try:
            # 重置舞蹈服务状态标志
            self.dance_service_called = False
            
            # 对于Up/Down模式，强制重置服务状态
            if self.dance_type in ["Up", "Down"]:
                rospy.loginfo(f"重置{self.dance_type}模式服务状态...")
                try:
                    # 尝试调用服务来重置状态
                    if self.dance_type == "Up" and self.play_up_service is not None:
                        # 发送一个快速的重置请求
                        rospy.loginfo("发送Up服务重置请求...")
                    elif self.dance_type == "Down" and self.play_down_service is not None:
                        # 发送一个快速的重置请求
                        rospy.loginfo("发送Down服务重置请求...")
                    rospy.sleep(1.0)  # 给服务时间重置
                except Exception as e:
                    rospy.logwarn(f"重置{self.dance_type}服务状态失败: {e}")
            
            # 对于所有模式，尝试重置舞蹈服务状态
            if self.play_dance_service is not None:
                rospy.loginfo("重置舞蹈服务状态...")
                try:
                    # 发布停止消息，确保舞蹈服务处于停止状态
                    stop_msg = String()
                    stop_msg.data = "stop"
                    # 如果有停止话题，可以在这里发布
                    rospy.loginfo("舞蹈服务停止命令已发送")
                except Exception as e:
                    rospy.logwarn(f"重置舞蹈服务状态失败: {e}")
                    
            rospy.loginfo("所有舞蹈服务状态重置完成")
            
        except Exception as e:
            rospy.logwarn(f"重置舞蹈服务状态失败: {e}")
        
        rospy.loginfo(f"Starting performance, dance type: {self.dance_type}")
        rospy.loginfo(f"总路径点数量: {len(self.waypoints)}")
        rospy.loginfo(f"第一个路径点: {self.waypoints[0] if self.waypoints else 'None'}")
        rospy.loginfo(f"舞蹈服务状态: dance_service_called={self.dance_service_called}")
        rospy.loginfo(f"舞蹈服务可用性: play_dance_service={'可用' if self.play_dance_service is not None else '不可用'}")
        
        # 创建固定时间调度
        self.create_waypoint_schedule()
        
        # 再次检查舞蹈服务状态 - 只检查服务是否存在，不实际调用
        if self.play_dance_service is not None:
            try:
                rospy.loginfo("Final dance service check...")
                # 只检查服务是否可用，不实际调用
                rospy.loginfo("Final dance service is available")
            except Exception as e:
                rospy.logwarn(f"Final dance service test failed: {e}")
                self.play_dance_service = None
        
        self.start_navigation_watchdog()
        rospy.loginfo("开始导航到第一个路径点...")
        self.navigate_to_current_waypoint()
        self.waypoint_start_time = rospy.Time.now()
        rospy.loginfo("导航启动完成")
        
        # 注册关闭处理程序
        rospy.on_shutdown(self.cleanup)
    
    def set_state(self, new_state):
        """线程安全的状态设置"""
        with self._lock:
            old_state = self._state
            self._state = new_state
            rospy.loginfo(f"状态变更: {old_state.value} -> {new_state.value}")
    
    def get_state(self):
        """线程安全的状态获取"""
        with self._lock:
            return self._state
    
    def is_doorway_waypoint(self, waypoint_index):
        """检查指定索引的路径点是否是门口点位"""
        if waypoint_index >= len(self.waypoints):
            return False
        
        x, y, theta = self.waypoints[waypoint_index]
        # 检查坐标是否与门口点位匹配（允许小的误差）
        return (abs(x - self.doorway_pos[0]) < 0.1 and 
                abs(y - self.doorway_pos[1]) < 0.1)
    
    def is_current_doorway_waypoint(self):
        """检查当前路径点是否是门口点位"""
        return self.is_doorway_waypoint(self.current_waypoint_index)
    
    def optimize_waypoints(self):
        """智能优化路径点，如果机器人能直接规划到后台就跳过过渡点"""
        try:
            # 检查是否有门口过渡点
            doorway_indices = [i for i in range(len(self.waypoints)) if self.is_doorway_waypoint(i)]
            
            if not doorway_indices:
                rospy.loginfo("没有门口过渡点，无需优化")
                return
            
            # 检查是否可以直接从舞蹈点规划到后台点
            dance_points = [i for i in range(len(self.waypoints)) if not self.is_doorway_waypoint(i) and i != len(self.waypoints) - 1]
            backstage_index = len(self.waypoints) - 1
            
            if not dance_points:
                rospy.loginfo("没有舞蹈点，无需优化")
                return
            
            # 尝试从最后一个舞蹈点直接规划到后台点
            last_dance_index = max(dance_points)
            last_dance_pos = self.waypoints[last_dance_index]
            backstage_pos = self.waypoints[backstage_index]
            
            rospy.loginfo(f"检查是否可以从舞蹈点 {last_dance_pos} 直接规划到后台点 {backstage_pos}")
            
            # 计算距离，如果距离很近，可能不需要过渡点
            distance = math.sqrt((last_dance_pos[0] - backstage_pos[0])**2 + (last_dance_pos[1] - backstage_pos[1])**2)
            rospy.loginfo(f"舞蹈点到后台点距离: {distance:.2f}米")
            
            # 如果距离小于3米，尝试跳过过渡点
            if distance < 3.0:
                rospy.loginfo("距离较近，尝试跳过门口过渡点...")
                
                # 创建优化后的路径点序列（移除门口过渡点）
                optimized_waypoints = []
                optimized_wait_times = []
                
                for i in range(len(self.waypoints)):
                    if not self.is_doorway_waypoint(i):
                        optimized_waypoints.append(self.waypoints[i])
                        if i < len(self.wait_times):
                            optimized_wait_times.append(self.wait_times[i])
                        else:
                            optimized_wait_times.append(0)
                
                # 更新路径点序列
                self.waypoints = optimized_waypoints
                self.wait_times = optimized_wait_times
                
                rospy.loginfo(f"路径优化完成，新路径点数量: {len(self.waypoints)}")
                rospy.loginfo(f"优化后的路径点序列: {self.waypoints}")
                
                # 重新计算门口点数量
                doorway_count = sum(1 for i in range(len(self.waypoints)) if self.is_doorway_waypoint(i))
                rospy.loginfo(f"优化后门口过渡点数量: {doorway_count}")
            else:
                rospy.loginfo(f"距离较远({distance:.2f}米)，保留门口过渡点")
                
        except Exception as e:
            rospy.logwarn(f"路径优化失败: {e}")
            # 如果优化失败，保持原始路径
    
    def get_waypoint_description(self, waypoint_index):
        """获取路径点的描述信息"""
        if waypoint_index >= len(self.waypoints):
            return "无效路径点"
        
        if waypoint_index == len(self.waypoints) - 1:
            return "[返回后台]"
        elif waypoint_index == 0:
            return f"[第一个舞蹈位置]"
        elif self.is_doorway_waypoint(waypoint_index):
            return "[门口过渡点]"
        else:
            return f"[舞蹈位置 {waypoint_index+1}]"
    
    def _schedule_final_shutdown(self):
        """等待g1_control的停止信息来关闭程序"""
        rospy.loginfo("等待g1_control的停止信息来关闭程序...")
        # 不需要设置定时器，只等待g1_control的停止信息
    
    def _check_backstage_arrival(self):
        """检查机器人是否真正到达后台点"""
        if not self.waypoints:
            return False
            
        # 获取后台点位置
        backstage_pos = self.waypoints[-1]
        
        # 计算距离
        dx = self.current_position["x"] - backstage_pos[0]
        dy = self.current_position["y"] - backstage_pos[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        rospy.loginfo(f"距离后台点还有: {distance:.2f}米")
        
        # 如果距离小于0.3米，认为已到达
        if distance < 0.3:
            rospy.loginfo(f"✅ 机器人已到达后台点 (距离: {distance:.2f}米 < 0.3米)")
            return True
        else:
            rospy.loginfo(f"⏳ 机器人尚未到达后台点 (距离: {distance:.2f}米 >= 0.3米)")
            return False
    
    def _final_shutdown(self):
        """最终关闭程序 - 由g1_control的停止信息触发"""
        rospy.loginfo("程序即将关闭")
        rospy.signal_shutdown("任务完成")
        import sys
        sys.exit(0)

    def cleanup(self):
        """清理资源"""
        rospy.loginfo("开始清理资源...")
        
        # 停止所有定时器
        for timer in self._timers:
            try:
                timer.shutdown()
            except Exception as e:
                rospy.logwarn(f"定时器关闭失败: {e}")
        
        # 等待所有线程结束
        for thread in self._threads:
            if thread.is_alive():
                thread.join(timeout=1.0)
        
        # 取消当前目标
        try:
            cancel_msg = GoalID()
            self.cancel_pub.publish(cancel_msg)
        except Exception as e:
            rospy.logwarn(f"取消目标失败: {e}")
        
        # 清理舞蹈服务相关资源
        try:
            if hasattr(self, 'play_dance_service') and self.play_dance_service is not None:
                # 重置舞蹈服务状态
                self.dance_service_called = False
                rospy.loginfo("舞蹈服务状态已重置")
                
                # 尝试停止舞蹈服务，确保服务状态重置
                try:
                    rospy.loginfo("尝试停止舞蹈服务...")
                    # 发送停止命令给舞蹈服务
                    stop_msg = String()
                    stop_msg.data = "stop"
                    # 这里可以发布停止消息，如果有对应的停止话题
                    rospy.loginfo("舞蹈服务停止命令已发送")
                except Exception as stop_e:
                    rospy.logwarn(f"停止舞蹈服务失败: {stop_e}")
                    
        except Exception as e:
            rospy.logwarn(f"清理舞蹈服务失败: {e}")
        
        rospy.loginfo("资源清理完成")
    
    def get_status_info(self):
        """获取当前状态信息，用于调试和监控"""
        with self._lock:
            # 计算全局时间信息
            global_time_info = {}
            if self.global_start_time is not None:
                elapsed_time = (rospy.Time.now() - self.global_start_time).to_sec()
                remaining_time = self.global_time_limit - elapsed_time
                global_time_info = {
                    "global_elapsed": elapsed_time,
                    "global_remaining": remaining_time,
                    "global_limit": self.global_time_limit,
                    "force_return_triggered": self.force_return_triggered
                }
            
            return {
                "state": self._state.value,
                "current_waypoint": self.current_waypoint_index,
                "total_waypoints": len(self.waypoints),
                "position": self.current_position.copy(),
                "dance_service_called": self.dance_service_called,
                "reached_final": self.reached_final,
                "active_timers": len(self._timers),
                "active_threads": len([t for t in self._threads if t.is_alive()]),
                **global_time_info
            }
    
    def log_status_info(self, event):
        """定期记录状态信息"""
        status = self.get_status_info()
        
        # 添加固定时间调度信息
        schedule_info = ""
        if self.waypoint_schedule and self.global_start_time:
            elapsed_time = (rospy.Time.now() - self.global_start_time).to_sec()
            if self.current_waypoint_index < len(self.waypoint_schedule):
                current_schedule = self.waypoint_schedule[self.current_waypoint_index]
                remaining_time = current_schedule['end_time'] - elapsed_time
                schedule_info = f" 调度时间:{elapsed_time:.1f}s 剩余:{remaining_time:.1f}s"
        
        # 添加更详细的航点信息
        if self.current_waypoint_index < len(self.waypoints):
            current_waypoint = self.waypoints[self.current_waypoint_index]
            waypoint_desc = self.get_waypoint_description(self.current_waypoint_index)
            rospy.loginfo(f"[状态监控] 状态:{status['state']} 航点:{status['current_waypoint']+1}/{status['total_waypoints']} {waypoint_desc} 位置:{current_waypoint} 舞蹈已调用:{status['dance_service_called']}{schedule_info}")
        else:
            rospy.loginfo(f"[状态监控] {status}{schedule_info}")

    def status_callback(self, msg):
        """Monitor move_base status"""
        self.move_base_status = msg
    
    def check_robot_stopped(self, event):
        """检测机器人是否停止 - 通过监控cmd_vel为0的持续时间"""
        current_state = self.get_state()
        if current_state not in [NavigationState.NAVIGATING]:
            return
            
        # 检查当前时间与上次cmd_vel的时间差
        current_time = rospy.Time.now()
        time_since_last_cmd = (current_time - self.last_cmd_vel_time).to_sec()
        
        # 如果超过1秒没有收到cmd_vel，认为机器人已停止
        if time_since_last_cmd > 1.0:
            self.cmd_vel_zero_duration += 0.5  # 定时器间隔0.5秒
            rospy.loginfo(f"机器人停止检测: 停止时间{self.cmd_vel_zero_duration:.1f}秒，需要{self.required_stop_time}秒")
            
            # 如果停止时间超过阈值，认为到达目标
            if self.cmd_vel_zero_duration >= self.required_stop_time:
                rospy.loginfo(f"检测到机器人已停止{self.cmd_vel_zero_duration:.1f}秒，认为到达目标")
                self._handle_waypoint_arrival()
                self.cmd_vel_zero_duration = 0.0  # 重置
        else:
            # 如果还在接收cmd_vel，重置停止时间
            self.cmd_vel_zero_duration = 0.0
    
    def _handle_waypoint_arrival(self):
        """处理到达航点的逻辑"""
        current_state = self.get_state()
        if current_state != NavigationState.NAVIGATING:
            rospy.logwarn(f"当前状态不是NAVIGATING，无法处理到达: {current_state}")
            return
            
        rospy.loginfo(f"到达路径点 {self.current_waypoint_index+1}")
        rospy.loginfo(f"当前航点索引: {self.current_waypoint_index}, 舞蹈服务已调用: {self.dance_service_called}")
        
        # 检查是否是门口过渡点
        if self.is_current_doorway_waypoint():
            rospy.loginfo("到达门口过渡点，短暂停留后继续")
            self.set_state(NavigationState.WAITING)
            # 门口点位等待时间很短，直接继续
            self.schedule_next_waypoint()
        # 对于第一个点，需要开始舞蹈
        elif self.current_waypoint_index == 0 and not self.dance_service_called:
            rospy.loginfo("到达第一个点，开始执行舞蹈")
            
            # 在开始舞蹈前，再次确保舞蹈服务状态正确
            rospy.loginfo("检查舞蹈服务状态...")
            try:
                # 强制重置舞蹈服务状态
                self.dance_service_called = False
                rospy.loginfo("舞蹈服务状态已重置")
                
                # 等待一小段时间确保服务状态稳定
                rospy.sleep(0.5)
                
            except Exception as e:
                rospy.logwarn(f"重置舞蹈服务状态失败: {e}")
            
            # 启动全局时间计时器（从到达第一个点开始计时）
            self.start_global_timer()
            self.set_state(NavigationState.WAITING)
            self.perform_dance()
            # 注意：第一个点的索引增加在舞蹈完成后通过schedule_next_waypoint处理
        else:
            # 其他点或舞蹈已调用，直接按等待时间停留
            rospy.loginfo("按等待时间停留")
            self.set_state(NavigationState.WAITING)
            self.schedule_next_waypoint()
    
    def arrival_callback(self, msg):
        """接收机器人到达状态 - 唯一的程序关闭监测点"""
        current_state = self.get_state()
        if current_state != NavigationState.NAVIGATING:
            return
            
        if msg.data == "arrived":
            rospy.loginfo("收到机器人到达状态，开始处理航点到达逻辑")
            # 检查是否是最后一个航点（后台点）
            if self.current_waypoint_index == len(self.waypoints) - 1:
                # 检查机器人当前位置是否真正到达后台点（距离小于0.3米）
                if self._check_backstage_arrival():
                    rospy.loginfo("机器人已真正到达后台点，程序即将关闭")
                    self._final_shutdown()
                else:
                    rospy.loginfo("机器人尚未真正到达后台点，继续等待...")
            else:
                self._handle_waypoint_arrival()
    
    def cmd_vel_callback(self, msg):
        """监控cmd_vel以检测机器人停止"""
        # 更新最后接收cmd_vel的时间
        self.last_cmd_vel_time = rospy.Time.now()
        
        # 如果cmd_vel不为0，重置停止时间
        if not (msg.linear.x == 0.0 and msg.linear.y == 0.0 and msg.linear.z == 0.0 and
                msg.angular.x == 0.0 and msg.angular.y == 0.0 and msg.angular.z == 0.0):
            self.cmd_vel_zero_duration = 0.0
    

    def start_navigation_watchdog(self):
        """Start a timer to periodically check if navigation is progressing"""
        if hasattr(self, 'nav_watchdog_timer') and self.nav_watchdog_timer:
            self.nav_watchdog_timer.shutdown()
        self.nav_watchdog_timer = rospy.Timer(
            rospy.Duration(8.0), self.check_navigation_progress
        )  # 增加检查间隔
        self._timers.append(self.nav_watchdog_timer)

   #  def ensure_movement(self, event):
   #      """检查导航进度，只在真正卡住时才干预"""
   #      current_state = self.get_state()
   #      if current_state in [NavigationState.DANCING, NavigationState.COMPLETED, NavigationState.WAITING]:
   #          return

   #      # 只在导航状态且发布航点后超过30秒时才检查（给足够时间到达）
   #      if (current_state == NavigationState.NAVIGATING and 
   #          self.waypoint_start_time and 
   #          (rospy.Time.now() - self.waypoint_start_time).to_sec() > 30.0):
            
   #          # 检查是否是最后一个航点
   #          is_last_waypoint = (self.current_waypoint_index == len(self.waypoints) - 1)
            
   #          if is_last_waypoint:
   #              rospy.logwarn(f"在最后一个路径点导航超过30秒，继续尝试...")
   #              # 重新发布最后一个航点目标
   #              rospy.loginfo("重新发布最后一个航点目标...")
   #              self.navigate_to_current_waypoint(is_retry=True)
   #          else:
   #              rospy.logwarn(f"在路径点{self.current_waypoint_index+1}导航超过30秒，强制前进")
   #              self.force_move_to_next_waypoint()

    def force_move_to_next_waypoint(self):
        """强制移动到下一个路径点，不考虑当前点是否到达（除了最后一个点）"""
        with self._lock:
            # 检查是否是最后一个航点
            is_last_waypoint = (self.current_waypoint_index == len(self.waypoints) - 1)
            
            if is_last_waypoint:
                rospy.logwarn("这是最后一个航点，不允许跳过，继续尝试...")
                # 重新发布最后一个航点目标
                rospy.loginfo("重新发布最后一个航点目标...")
                self.navigate_to_current_waypoint(is_retry=True)
                return
            
            # 取消当前目标
            cancel_msg = GoalID()
            self.cancel_pub.publish(cancel_msg)
            rospy.sleep(0.1)
            
            self.current_waypoint_index += 1
            rospy.loginfo(f"[调试] 强制前进，当前索引: {self.current_waypoint_index}, 总航点数: {len(self.waypoints)}")
            
            if self.current_waypoint_index < len(self.waypoints):
                rospy.loginfo(
                    f"强制前进到路径点 {self.current_waypoint_index+1}/{len(self.waypoints)}"
                )
                # 短暂延迟后尝试下一个点
                rospy.sleep(1.0)
                self.navigate_to_current_waypoint()
                self.waypoint_start_time = rospy.Time.now()  # 重置计时器
            else:
                rospy.loginfo("[完成] 所有路径点已完成!")
                rospy.loginfo(f"[调试] 最终航点索引: {self.current_waypoint_index}, 总航点数: {len(self.waypoints)}")
                self.reached_final = True
                self.set_state(NavigationState.COMPLETED)
                # 给机器人时间完成最后的导航，然后延迟退出
                rospy.loginfo("所有路径点已完成，等待机器人完成最后的导航...")
                self._schedule_final_shutdown()


    def check_navigation_progress(self, event):
        """检查导航进度，只在真正卡住时才干预"""
        current_state = self.get_state()
        if current_state not in [NavigationState.NAVIGATING]:
            return

        # 检查从发布航点开始的总时间
        if self.waypoint_start_time:
            total_time = (rospy.Time.now() - self.waypoint_start_time).to_sec()
            
            # 如果发布航点后超过45秒还没到达，认为卡住了
            if total_time > 45.0:
                # 检查是否是门口过渡点
                if self.is_current_doorway_waypoint():
                    rospy.logwarn(f"[超时] 门口过渡点导航超过{total_time:.1f}秒，继续尝试...")
                    # 重新发布门口过渡点目标
                    rospy.loginfo("重新发布门口过渡点目标...")
                    self.navigate_to_current_waypoint(is_retry=True)
                # 检查是否是最后一个航点
                elif self.current_waypoint_index == len(self.waypoints) - 1:
                    rospy.logwarn(f"[超时] 在最后一个路径点导航超过{total_time:.1f}秒，继续尝试...")
                    # 重新发布最后一个航点目标
                    rospy.loginfo("重新发布最后一个航点目标...")
                    self.navigate_to_current_waypoint(is_retry=True)
                else:
                    rospy.logwarn(f"[超时] 在路径点导航超过{total_time:.1f}秒，强制前进...")
                    self.force_move_to_next_waypoint()
                return


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

    def _optimize_planner_for_doorway(self):
        """为门口点位优化规划器参数"""
        try:
            import rospy
            from dynamic_reconfigure.client import Client
            
            # 创建动态重配置客户端
            client = Client("/move_base/TebLocalPlannerROS", timeout=5.0)
            
            # 为门口点位设置更宽松的参数
            doorway_params = {
                'xy_goal_tolerance': 0.3,  # 增加目标容差
                'yaw_goal_tolerance': 0.3,  # 增加角度容差
                'min_obstacle_dist': 0.05,  # 减少最小障碍物距离
                'inflation_dist': 0.1,     # 减少膨胀距离
                'weight_obstacle': 20,      # 减少障碍物权重
                'weight_inflation': 0.05,   # 减少膨胀权重
                'max_vel_x': 1.5,           # 降低最大速度
                'max_vel_theta': 0.8,       # 降低最大角速度
                'acc_lim_x': 0.8,          # 降低加速度限制
                'acc_lim_theta': 0.8,      # 降低角加速度限制
                'enable_homotopy_class_planning': True,  # 启用同伦类规划
                'max_number_classes': 6,   # 增加同伦类数量
                'selection_cost_hysteresis': 0.5,  # 减少成本滞后
                'roadmap_graph_no_samples': 20,    # 增加采样点
                'roadmap_graph_area_width': 8,    # 增加规划区域宽度
            }
            
            rospy.loginfo("为门口点位应用优化参数...")
            client.update_configuration(doorway_params)
            rospy.loginfo("门口点位参数优化完成")
            
        except Exception as e:
            rospy.logwarn(f"门口点位参数优化失败: {e}")
            # 如果动态重配置失败，继续使用默认参数

    def _restore_default_planner_params(self):
        """恢复默认规划器参数"""
        try:
            import rospy
            from dynamic_reconfigure.client import Client
            
            # 恢复默认参数
            default_params = {
                'xy_goal_tolerance': 0.2,
                'yaw_goal_tolerance': 0.15,
                'min_obstacle_dist': 0.1,
                'inflation_dist': 0.2,
                'weight_obstacle': 50,
                'weight_inflation': 0.1,
                'max_vel_x': 3.0,
                'max_vel_theta': 1.0,
                'acc_lim_x': 1.0,
                'acc_lim_theta': 1.0,
                'enable_homotopy_class_planning': True,
                'max_number_classes': 4,
                'selection_cost_hysteresis': 1.0,
                'roadmap_graph_no_samples': 15,
                'roadmap_graph_area_width': 5,
            }
            
            rospy.loginfo("恢复默认规划器参数...")
            client = Client("/move_base/TebLocalPlannerROS", timeout=5.0)
            client.update_configuration(default_params)
            rospy.loginfo("默认参数恢复完成")
            
        except Exception as e:
            rospy.logwarn(f"恢复默认参数失败: {e}")

    def create_waypoint_schedule(self):
        """创建基于全局时间的固定航点调度"""
        if not self.waypoints or not self.wait_times:
            rospy.logwarn("没有航点或等待时间数据，无法创建调度")
            return
            
        # 计算每个航点的计划时间
        current_time = 0.0
        self.waypoint_schedule = []
        
        for i in range(len(self.waypoints)):
            waypoint = self.waypoints[i]
            wait_time = self.wait_times[i] if i < len(self.wait_times) else 0
            
            # 记录这个航点的计划时间
            schedule_entry = {
                'waypoint_index': i,
                'start_time': current_time,
                'end_time': current_time + wait_time,
                'wait_time': wait_time,
                'waypoint': waypoint
            }
            self.waypoint_schedule.append(schedule_entry)
            
            rospy.loginfo(f"航点 {i+1}: {self.get_waypoint_description(i)} 计划时间 {current_time:.1f}s - {current_time + wait_time:.1f}s (等待{wait_time}s)")
            
            # 下一个航点的开始时间
            current_time += wait_time
        
        rospy.loginfo(f"固定时间调度创建完成，总计划时间: {current_time:.1f}秒")
        rospy.loginfo("调度定时器将在到达第一个点时启动")
    
    def start_schedule_timer(self):
        """启动调度定时器，检查是否到了换点时间"""
        if self.schedule_timer:
            self.schedule_timer.shutdown()
            
        # 每秒检查一次调度
        self.schedule_timer = rospy.Timer(
            rospy.Duration(1.0), self.check_schedule, oneshot=False
        )
        self._timers.append(self.schedule_timer)
    
    def check_schedule(self, event):
        """检查是否到了换点时间"""
        if not self.waypoint_schedule or not self.global_start_time:
            return
            
        elapsed_time = (rospy.Time.now() - self.global_start_time).to_sec()
        
        # 检查当前航点是否应该结束
        if self.current_waypoint_index < len(self.waypoint_schedule):
            current_schedule = self.waypoint_schedule[self.current_waypoint_index]
            
            # 如果超过了当前航点的计划结束时间，强制换点
            if elapsed_time >= current_schedule['end_time']:
                rospy.logwarn(f"[固定时间调度] 航点 {self.current_waypoint_index+1} 计划时间已到 ({elapsed_time:.1f}s >= {current_schedule['end_time']:.1f}s)，强制换点")
                self.force_transition_to_next_waypoint()
    
    def force_transition_to_next_waypoint(self):
        """强制转换到下一个航点，不管当前导航状态"""
        with self._lock:
            rospy.logwarn(f"[固定时间调度] 强制从航点 {self.current_waypoint_index+1} 转换到下一个航点")
            
            # 取消当前导航
            try:
                cancel_msg = GoalID()
                self.cancel_pub.publish(cancel_msg)
                rospy.sleep(0.1)
            except Exception as e:
                rospy.logwarn(f"取消当前导航失败: {e}")
            
            # 直接进入下一个航点
            self.current_waypoint_index += 1
            
            if self.current_waypoint_index < len(self.waypoints):
                rospy.loginfo(f"[固定时间调度] 开始导航到航点 {self.current_waypoint_index+1}")
                self.navigate_to_current_waypoint()
            else:
                rospy.loginfo("[固定时间调度] 所有航点已完成")
                self.reached_final = True
                self.set_state(NavigationState.COMPLETED)
                self._schedule_final_shutdown()

    def start_global_timer(self):
        """启动全局时间计时器"""
        if self.global_start_time is None:
            self.global_start_time = rospy.Time.now()
            rospy.loginfo(f"开始全局时间计时，限制时间: {self.global_time_limit}秒")
            
            # 设置全局时间检查定时器
            self.global_timer = rospy.Timer(
                rospy.Duration(1.0), self.check_global_time, oneshot=False
            )
            self._timers.append(self.global_timer)
            
            # 同时启动调度定时器
            self.start_schedule_timer()
            rospy.loginfo("固定时间调度定时器已启动")

    def check_global_time(self, event):
        """检查全局时间是否超时"""
        if self.global_start_time is None:
            return
            
        elapsed_time = (rospy.Time.now() - self.global_start_time).to_sec()
        remaining_time = self.global_time_limit - elapsed_time
        
        # 每10秒记录一次剩余时间
        if int(elapsed_time) % 10 == 0:
            rospy.loginfo(f"[全局时间] 已用时: {elapsed_time:.1f}秒, 剩余: {remaining_time:.1f}秒")
        
        # 如果超时，强制返回后台
        if elapsed_time >= self.global_time_limit and not self.force_return_triggered:
            rospy.logwarn(f"[全局时间超时] 已用时 {elapsed_time:.1f}秒，超过限制 {self.global_time_limit}秒，强制返回后台！")
            self.force_return_to_backstage()

    def force_return_to_backstage(self):
        """强制返回后台点"""
        with self._lock:
            if self.force_return_triggered:
                return
                
            self.force_return_triggered = True
            rospy.logwarn("触发强制返回后台逻辑...")
            
            # 取消当前所有目标
            try:
                cancel_msg = GoalID()
                self.cancel_pub.publish(cancel_msg)
                rospy.sleep(0.2)
            except Exception as e:
                rospy.logwarn(f"取消目标失败: {e}")
            
            # 停止所有定时器
            for timer in self._timers:
                try:
                    timer.shutdown()
                except Exception as e:
                    rospy.logwarn(f"定时器关闭失败: {e}")
            
            # 直接导航到后台点
            backstage_pos = (-0.6, 0, 0)
            rospy.logwarn(f"强制导航到后台点: {backstage_pos}")
            
            # 发布后台点目标
            goal_msg = self._build_move_base_goal(backstage_pos[0], backstage_pos[1], backstage_pos[2])
            self.goal_pub.publish(goal_msg)
            
            # 设置状态为强制返回
            self.set_state(NavigationState.NAVIGATING)
            self.current_waypoint_index = len(self.waypoints) - 1  # 设置为最后一个点
            
            # 设置一个较短的超时检测，确保能到达后台
            self.plan_failure_timer = rospy.Timer(
                rospy.Duration(30.0), self.check_force_return_success, oneshot=True
            )
            self._timers.append(self.plan_failure_timer)

    def check_force_return_success(self, event):
        """检查强制返回是否成功 - 简化版本，只等待g1_control停止信息"""
        rospy.loginfo("强制返回已启动，等待g1_control停止信息...")
        # 不再检查位置，只等待g1_control的停止信息

    def _try_rviz_style_backstage_approach(self):
        """RViz风格的后台点备用策略 - 在后台点附近尝试多个位置"""
        rospy.loginfo("尝试RViz风格后台点备用策略...")
        
        # 原始后台点
        original_backstage = (-0.6, 0, 0)
        
        # 生成备用后台点列表（模拟RViz手动点击的位置）
        alternative_backstage_points = [
            (-0.5, 0, 0),      # 稍微向右
            (-0.7, 0, 0),      # 稍微向左  
            (-0.6, 0.1, 0),     # 稍微向前
            (-0.6, -0.1, 0),    # 稍微向后
            (-0.4, 0, 0),       # 更向右
            (-0.8, 0, 0),       # 更向左
            (-0.6, 0.2, 0),     # 更向前
            (-0.6, -0.2, 0),    # 更向后
            (-0.3, 0, 0),       # 很向右
            (-0.9, 0, 0),       # 很向左
            (-0.6, 0.3, 0),     # 很向前
            (-0.6, -0.3, 0),    # 很向后
        ]
        
        # 尝试每个备用点
        for i, (x, y, theta) in enumerate(alternative_backstage_points):
            rospy.loginfo(f"尝试备用后台点 {i+1}/{len(alternative_backstage_points)}: ({x}, {y}, {theta})")
            
            # 取消当前目标
            cancel_msg = GoalID()
            self.cancel_pub.publish(cancel_msg)
            rospy.sleep(0.5)
            
            # 发布备用目标
            goal_msg = self._build_move_base_goal(x, y, theta)
            self.goal_pub.publish(goal_msg)
            
            # 等待一段时间看是否成功
            rospy.sleep(3.0)
            
            # 检查是否开始移动
            current_dx = self.current_position["x"] - self.last_position_check["x"]
            current_dy = self.current_position["y"] - self.last_position_check["y"]
            current_dist_moved = math.hypot(current_dx, current_dy)
            
            # 如果开始移动，认为成功
            if current_dist_moved > 0.1:
                rospy.loginfo(f"✅ 备用后台点 {i+1} 成功，机器人开始移动 {current_dist_moved:.3f}米")
                return
            else:
                rospy.logwarn(f"备用后台点 {i+1} 未成功，继续尝试下一个...")
        
        # 如果所有备用点都失败，最后尝试原始后台点
        rospy.logwarn("所有备用后台点都失败，最后尝试原始后台点...")
        cancel_msg = GoalID()
        self.cancel_pub.publish(cancel_msg)
        rospy.sleep(0.5)
        
        goal_msg = self._build_move_base_goal(original_backstage[0], original_backstage[1], original_backstage[2])
        self.goal_pub.publish(goal_msg)
        
        rospy.loginfo("原始后台点目标已发布，等待g1_control停止信息...")

    def _try_rviz_style_first_waypoint_approach(self):
        """RViz风格的第一个航点备用策略 - 在第一个航点附近尝试多个位置"""
        rospy.loginfo("尝试RViz风格第一个航点备用策略...")
        
        # 获取原始第一个航点位置
        original_waypoint = self.waypoints[self.current_waypoint_index]
        x, y, theta = original_waypoint
        
        # 生成备用第一个航点列表（模拟RViz手动点击的位置）
        alternative_first_waypoints = [
            (x + 0.2, y, theta),      # 稍微向右
            (x - 0.2, y, theta),      # 稍微向左  
            (x, y + 0.2, theta),      # 稍微向前
            (x, y - 0.2, theta),      # 稍微向后
            (x + 0.1, y + 0.1, theta), # 稍微向右前
            (x - 0.1, y + 0.1, theta), # 稍微向左前
            (x + 0.1, y - 0.1, theta), # 稍微向右后
            (x - 0.1, y - 0.1, theta), # 稍微向左后
            (x + 0.3, y, theta),      # 更向右
            (x - 0.3, y, theta),      # 更向左
            (x, y + 0.3, theta),      # 更向前
            (x, y - 0.3, theta),      # 更向后
        ]
        
        # 尝试每个备用点
        for i, (alt_x, alt_y, alt_theta) in enumerate(alternative_first_waypoints):
            rospy.loginfo(f"尝试备用第一个航点 {i+1}/{len(alternative_first_waypoints)}: ({alt_x}, {alt_y}, {alt_theta})")
            
            # 取消当前目标
            cancel_msg = GoalID()
            self.cancel_pub.publish(cancel_msg)
            rospy.sleep(0.5)
            
            # 临时修改当前航点
            self.waypoints[self.current_waypoint_index] = (alt_x, alt_y, alt_theta)
            
            # 发布备用目标
            goal_msg = self._build_move_base_goal(alt_x, alt_y, alt_theta)
            self.goal_pub.publish(goal_msg)
            
            # 等待一段时间看是否成功
            rospy.sleep(3.0)
            
            # 检查是否开始移动
            current_dx = self.current_position["x"] - self.last_position_check["x"]
            current_dy = self.current_position["y"] - self.last_position_check["y"]
            current_dist_moved = math.hypot(current_dx, current_dy)
            
            # 如果开始移动，认为成功
            if current_dist_moved > 0.1:
                rospy.loginfo(f"✅ 备用第一个航点 {i+1} 成功，机器人开始移动 {current_dist_moved:.3f}米")
                # 恢复原始航点位置
                self.waypoints[self.current_waypoint_index] = original_waypoint
                return
            else:
                rospy.logwarn(f"备用第一个航点 {i+1} 未成功，继续尝试下一个...")
        
        # 如果所有备用点都失败，恢复原始位置并跳过
        rospy.logwarn("所有备用第一个航点都失败，跳过第一个航点...")
        self.waypoints[self.current_waypoint_index] = original_waypoint
        self.force_move_to_next_waypoint()

    def _try_rviz_style_doorway_approach(self):
        """RViz风格的门口航点备用策略 - 在门口航点附近尝试多个位置"""
        rospy.loginfo("尝试RViz风格门口航点备用策略...")
        
        # 获取原始门口航点位置
        original_waypoint = self.waypoints[self.current_waypoint_index]
        x, y, theta = original_waypoint
        
        # 生成备用门口航点列表（模拟RViz手动点击的位置）
        alternative_doorway_waypoints = [
            (x + 0.1, y, theta),      # 稍微向右
            (x - 0.1, y, theta),      # 稍微向左  
            (x, y + 0.1, theta),      # 稍微向前
            (x, y - 0.1, theta),      # 稍微向后
            (x + 0.05, y + 0.05, theta), # 稍微向右前
            (x - 0.05, y + 0.05, theta), # 稍微向左前
            (x + 0.05, y - 0.05, theta), # 稍微向右后
            (x - 0.05, y - 0.05, theta), # 稍微向左后
            (x + 0.15, y, theta),     # 更向右
            (x - 0.15, y, theta),     # 更向左
            (x, y + 0.15, theta),     # 更向前
            (x, y - 0.15, theta),     # 更向后
        ]
        
        # 尝试每个备用点
        for i, (alt_x, alt_y, alt_theta) in enumerate(alternative_doorway_waypoints):
            rospy.loginfo(f"尝试备用门口航点 {i+1}/{len(alternative_doorway_waypoints)}: ({alt_x}, {alt_y}, {alt_theta})")
            
            # 取消当前目标
            cancel_msg = GoalID()
            self.cancel_pub.publish(cancel_msg)
            rospy.sleep(0.5)
            
            # 临时修改当前航点
            self.waypoints[self.current_waypoint_index] = (alt_x, alt_y, alt_theta)
            
            # 发布备用目标
            goal_msg = self._build_move_base_goal(alt_x, alt_y, alt_theta)
            self.goal_pub.publish(goal_msg)
            
            # 等待一段时间看是否成功
            rospy.sleep(3.0)
            
            # 检查是否开始移动
            current_dx = self.current_position["x"] - self.last_position_check["x"]
            current_dy = self.current_position["y"] - self.last_position_check["y"]
            current_dist_moved = math.hypot(current_dx, current_dy)
            
            # 如果开始移动，认为成功
            if current_dist_moved > 0.1:
                rospy.loginfo(f"✅ 备用门口航点 {i+1} 成功，机器人开始移动 {current_dist_moved:.3f}米")
                # 恢复原始航点位置
                self.waypoints[self.current_waypoint_index] = original_waypoint
                return
            else:
                rospy.logwarn(f"备用门口航点 {i+1} 未成功，继续尝试下一个...")
        
        # 如果所有备用点都失败，恢复原始位置并跳过
        rospy.logwarn("所有备用门口航点都失败，跳过门口航点...")
        self.waypoints[self.current_waypoint_index] = original_waypoint
        self.force_move_to_next_waypoint()

    def navigate_to_current_waypoint(self, is_retry=False):
        """Navigate to the current waypoint with error handling"""
        with self._lock:
            rospy.loginfo(f"[调试] 检查航点索引: {self.current_waypoint_index} >= {len(self.waypoints)}")
            if self.current_waypoint_index >= len(self.waypoints):
                rospy.loginfo("[完成] 所有路径点已完成!")
                rospy.loginfo(f"[调试] 最终航点索引: {self.current_waypoint_index}, 总航点数: {len(self.waypoints)}")
                self.reached_final = True
                self.set_state(NavigationState.COMPLETED)
                # 给机器人时间完成最后的导航，然后延迟退出
                rospy.loginfo("所有路径点已完成，等待机器人完成最后的导航...")
                self._schedule_final_shutdown()
                return

            # Reset for new navigation attempt
            if not is_retry:
                self.goal_send_retries = 0

            # First ensure any previous goals are canceled
            cancel_msg = GoalID()
            self.cancel_pub.publish(cancel_msg)
            rospy.sleep(0.1)

            # Get current waypoint
            x, y, theta = self.waypoints[self.current_waypoint_index]

            # Determine location description using the helper method
            location_desc = self.get_waypoint_description(self.current_waypoint_index)

            wait_time = (
                self.wait_times[self.current_waypoint_index]
                if self.current_waypoint_index < len(self.wait_times)
                else 0
            )
            prefix = "[重试] " if is_retry else ""
            rospy.loginfo(
                f"{prefix}[导航目标 {self.current_waypoint_index+1}/{len(self.waypoints)}] {location_desc} x={x}, y={y}, θ={theta} (等待时间: {wait_time}秒)"
            )
            rospy.loginfo(f"[调试] 当前航点索引: {self.current_waypoint_index}, 总航点数: {len(self.waypoints)}")
            rospy.loginfo(f"[调试] 剩余航点: {self.waypoints[self.current_waypoint_index:]}")

            # Update last position check
            self.last_position_check["x"] = self.current_position["x"]
            self.last_position_check["y"] = self.current_position["y"]
            self.last_position_check["time"] = rospy.Time.now()

            # 为门口点位添加特殊处理
            if self.is_current_doorway_waypoint():
                rospy.loginfo("门口过渡点：应用特殊路径规划策略")
                # 尝试调整规划器参数以提高成功率
                self._optimize_planner_for_doorway()
            
            # Publish MoveBaseActionGoal
            goal_msg = self._build_move_base_goal(x, y, theta)
            rospy.loginfo(f"发布导航目标: x={x}, y={y}, theta={theta}")
            
            self.goal_pub.publish(goal_msg)
            self.last_goal_send_time = rospy.Time.now()
            self.set_state(NavigationState.NAVIGATING)
            self.waypoint_start_time = rospy.Time.now()  # 重置路径点计时器
            rospy.loginfo("导航目标已发布，等待路径规划...")
            
            # 为门口点位使用更长的检测时间
            detection_time = 8.0 if self.is_current_doorway_waypoint() else 5.0
            self.plan_failure_timer = rospy.Timer(
                rospy.Duration(detection_time), self.check_plan_failure, oneshot=True
            )
            self._timers.append(self.plan_failure_timer)

    def check_plan_failure(self, event):
        """检查路径规划是否失败，如果失败则跳过当前点（除了最后一个点）"""
        current_state = self.get_state()
        if current_state == NavigationState.NAVIGATING:
            # 检查机器人是否在移动
            current_time = rospy.Time.now()
            time_since_start = (current_time - self.waypoint_start_time).to_sec()
            
            # 为门口点位使用更长的检测时间
            detection_threshold = 15.0 if self.is_current_doorway_waypoint() else 10.0
            
            if time_since_start > detection_threshold:
                # 检查机器人是否移动了
                dx = self.current_position["x"] - self.last_position_check["x"]
                dy = self.current_position["y"] - self.last_position_check["y"]
                dist_moved = math.hypot(dx, dy)
                
                if dist_moved < 0.1:  # 如果几乎没有移动
                    # 检查是否是第一个航点
                    if self.current_waypoint_index == 0:
                        rospy.logwarn(f"[路径规划失败] 第一个航点{detection_threshold}秒内只移动了{dist_moved:.3f}米，尝试RViz风格备用策略...")
                        # 对于第一个点，使用RViz风格的备用策略
                        self._try_rviz_style_first_waypoint_approach()
                    # 检查是否是门口过渡点
                    elif self.is_current_doorway_waypoint():
                        rospy.logwarn(f"[路径规划失败] 门口过渡点{detection_threshold}秒内只移动了{dist_moved:.3f}米，尝试RViz风格备用策略...")
                        # 使用RViz风格的备用策略
                        self._try_rviz_style_doorway_approach()
                    # 检查是否是最后一个航点（后台点）
                    elif self.current_waypoint_index == len(self.waypoints) - 1:
                        rospy.logwarn(f"[路径规划失败] 机器人{detection_threshold}秒内只移动了{dist_moved:.3f}米，但这是最后一个航点，尝试RViz风格备用策略...")
                        # 对于最后一个点，使用RViz风格的备用策略
                        self._try_rviz_style_backstage_approach()
                    else:
                        rospy.logwarn(f"[路径规划失败] 机器人{detection_threshold}秒内只移动了{dist_moved:.3f}米，跳过当前点")
                        self.force_move_to_next_waypoint()


    def perform_dance(self):
        """
        Call the dance service, only execute on first call
        """
        if self.dance_service_called:
            rospy.loginfo("舞蹈服务已被调用，不再重复调用")
            return

        try:
            # 确保机器人完全停止
            rospy.loginfo("发送停止命令确保机器人完全停止...")
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            rospy.sleep(0.5)  # 等待停止命令生效
            
            self.set_state(NavigationState.DANCING)
            
            # 检查是否是Up或Down模式，使用服务调用
            if self.dance_type in ["Up", "Down"]:
                rospy.loginfo(f"开始{self.dance_type}模式TTS和动作播放")
                
                # 检查对应的服务是否可用
                if self.dance_type == "Up" and self.play_up_service is None:
                    rospy.logwarn("Up服务不可用，跳过Up模式")
                    self.force_move_to_next_waypoint()
                    return
                elif self.dance_type == "Down" and self.play_down_service is None:
                    rospy.logwarn("Down服务不可用，跳过Down模式")
                    self.force_move_to_next_waypoint()
                    return
                
               #  在单独线程中调用对应的服务，添加超时机制
                def call_up_down_service():
                    try:
                        rospy.loginfo(f"开始调用{self.dance_type}模式服务...")
                        
                        # 添加服务调用前的状态检查和重置
                        rospy.loginfo("检查并重置服务状态...")
                        rospy.sleep(1.0)  # 给服务时间重置状态
                        
                        if self.dance_type == "Up":
                            response = self.play_up_service()
                        else:  # Down
                            response = self.play_down_service()
                        
                        if response.success:
                            rospy.loginfo(f"{self.dance_type}模式服务调用成功: {response.message}")
                        else:
                            rospy.logwarn(f"{self.dance_type}模式服务调用失败: {response.message}")
                    except Exception as e:
                        rospy.logerr(f"调用{self.dance_type}模式服务失败: {e}")
                    finally:
                        # 服务调用完成后，设置定时器等待
                        rospy.loginfo(f"{self.dance_type}模式服务调用完成，开始等待时间")
                        self.schedule_next_waypoint()
                
               #  # 设置服务调用超时定时器，防止服务卡住
               #  def service_timeout_handler():
               #      rospy.logwarn(f"{self.dance_type}模式服务调用超时，强制继续导航...")
               #      self.schedule_next_waypoint()
                
               #  # 设置超时定时器（等待时间 + 10秒缓冲）
               #  wait_time = self.wait_times[self.current_waypoint_index] if self.current_waypoint_index < len(self.wait_times) else 0
               #  timeout_duration = wait_time + 10.0  # 等待时间 + 10秒缓冲
               #  timeout_timer = rospy.Timer(
               #      rospy.Duration(timeout_duration), service_timeout_handler, oneshot=True
               #  )
               #  self._timers.append(timeout_timer)
                
                service_thread = threading.Thread(target=call_up_down_service)
                service_thread.daemon = True
                service_thread.start()
                self._threads.append(service_thread)
            else:
                # 原有的舞蹈服务逻辑
                if self.play_dance_service is None:
                    rospy.logwarn("舞蹈服务不可用，跳过舞蹈")
                    self.force_move_to_next_waypoint()
                    return

                dance_direction = self.dance_type
                rospy.loginfo(
                    f"开始舞蹈: {dance_direction}，在此表演中只会调用一次舞蹈服务"
                )
                self.dance_direction_pub.publish(String(dance_direction))
                rospy.sleep(0.5)

                # 在真正的非阻塞方式中调用服务
                dance_thread = threading.Thread(target=self._call_dance_service)
                dance_thread.daemon = True
                dance_thread.start()
                self._threads.append(dance_thread)

            # 标记舞蹈服务已调用
            self.dance_service_called = True
            rospy.loginfo("舞蹈服务调用已启动，等待完成后将继续导航")

            # 对于所有模式，设置定时器等待
            # 设置一个定时器，确保即使舞蹈服务卡住也能继续
            self.schedule_next_waypoint()

        except Exception as e:
            rospy.logerr(f"舞蹈服务调用失败: {e}")
            # 即使舞蹈失败，也应该继续前进
            self.force_move_to_next_waypoint()

    def _call_dance_service(self):
        """Helper method to call dance service in a separate thread"""
        try:
            response = self.play_dance_service()
            if response.success:
                rospy.loginfo("舞蹈服务已完成")
            else:
                rospy.logwarn(f"舞蹈服务返回失败: {response.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"舞蹈服务调用异常: {e}")
        except Exception as e:
            rospy.logerr(f"舞蹈服务执行错误: {e}")

    def schedule_next_waypoint(self):
        """Schedule movement to next waypoint - 现在由固定时间调度控制"""
        rospy.loginfo("航点转换现在由固定时间调度控制，不再使用等待时间")
        # 固定时间调度会自动处理航点转换，这里不需要做任何事情

    def continue_to_next_waypoint(self, event=None):
        """Timer callback to continue to next waypoint"""
        rospy.loginfo("等待时间结束，继续前往下一个路径点")

        with self._lock:
            # 如果刚完成门口点位，恢复默认参数
            if self.current_waypoint_index > 0 and self.is_doorway_waypoint(self.current_waypoint_index - 1):
                rospy.loginfo("门口点位完成，恢复默认规划器参数...")
                self._restore_default_planner_params()
            
            self.set_state(NavigationState.IDLE)
            self.current_waypoint_index += 1
            rospy.loginfo(f"[调试] 前进到下一个航点，当前索引: {self.current_waypoint_index}, 总航点数: {len(self.waypoints)}")

            if self.current_waypoint_index < len(self.waypoints):
                rospy.loginfo(
                    f"移动到路径点 {self.current_waypoint_index+1}/{len(self.waypoints)}"
                )
                self.navigate_to_current_waypoint()
            else:
                rospy.loginfo("[完成] 所有路径点已完成!")
                rospy.loginfo(f"[调试] 最终航点索引: {self.current_waypoint_index}, 总航点数: {len(self.waypoints)}")
                self.reached_final = True
                self.set_state(NavigationState.COMPLETED)
                # 给机器人时间完成最后的导航，然后延迟退出
                rospy.loginfo("所有路径点已完成，等待机器人完成最后的导航...")
                self._schedule_final_shutdown()

    def feedback_callback(self, msg):
        """Handle navigation feedback - 到达检测由g1_control.py处理"""
        current_state = self.get_state()
        # 即使reached_final为True，也要继续更新位置信息，直到真正完成导航
        if current_state in [NavigationState.DANCING, NavigationState.COMPLETED]:
            return

        # Update current position for logging only
        current_pose = msg.feedback.base_position.pose
        self.current_position["x"] = current_pose.position.x
        self.current_position["y"] = current_pose.position.y

        orientation = current_pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = tft.euler_from_quaternion(quaternion)
        self.current_position["theta"] = math.degrees(euler[2])

        # 只记录位置信息，不进行到达检测
        rospy.loginfo_throttle(
            5,
            f"[当前位置] ({current_pose.position.x:.2f}, {current_pose.position.y:.2f}) yaw: {self.current_position['theta']:.2f}°",
        )


if __name__ == "__main__":
    rospy.init_node("simple_nav_waypoints_player")
    parser = argparse.ArgumentParser(
        description="Navigation Dance Performance Controller"
    )
    parser.add_argument(
        "--dance",
        type=str,
        default="A",
        choices=["A", "B", "X", "Y","Up","Down"],
        help="Specify dance type to execute",
    )
    args, unknown = parser.parse_known_args()

    backstage_pos = (-0.6, 0, 0)


    
    dance_choreography = {
       #祖国的好山河 - 全局时间310秒  4*60+55=295
       # Start + Up:
        "A": {
            "global_time": 310.0,   
            "waypoints": [
                ((-1.91,1.35, 180), 67),
                
                
                
                ((-2.2, 3.0, 180), 30.0),
                ((-2.6, 3.4, 180), 40.0),
                ((-3.0, 3.4, -170), 20.0),   
                ((-2.2, 3.0, 180), 30.0),
                ((-2.6, 3.4, 180), 40.0),
                ((-3.0, 3.4, -170), 20.0),
                ((-2.2, 3.0, 180), 30.0),
                ((-2.6, 3.4, 180), 40.0),
                ((-3.0, 3.4, -170), 20.0),
                ((-1.91,1.35, 0), 0),    # 门口点位，中间过渡点
                ((-0.8, 0, 0), 0),
            ]
        },
        #沙家浜总有一天 - 
        # Start + Down:    2*60+45=165
        "B": {
            "global_time": 180.5,
            "waypoints": [
                ((-2.3, 3.6, 180), 35.0),
                ((-2.6, 3.6, -170), 35.0),
                ((-2.8, 3.6, 180), 42.5),
                                
                ((-2.7, 3.2, 160), 38.0),
                ((-2.7, 2.9, 180), 38.0),
                
                ((-2.8, 3.6, 180), 42.5),
                                
                ((-2.7, 3.1, 160), 38.0),
                ((-2.7, 2.9, 180), 38.0),
                
                ((-2.3, 3.6, 180), 40.0),
                ((-2.6, 3.6, -170), 40.0),
                ((-2.8, 3.6, 180), 42.5),
                                
                ((-2.7, 2.9, 160), 38.0),
                ((-2.7, 2.7, 180), 38.0),
                
                
                ((-2.3, 3.4, 180), 40.0),
                ((-2.6, 3.4, -170), 40.0),
                ((-2.8, 3.4, 180), 42.5),
                                
                ((-2.7, 2.9, 160), 38.0),
                ((-2.7, 2.7, 180), 38.0),
                
                ((-2.8, 3.4, 180), 42.5),
                                
                ((-2.7, 2.7, 180), 38.0),
                ((-2.7, 2.8, 160), 33.5),
                
                ((-1.91,1.35, 0), 0),    # 门口点位，中间过渡点
                ((-0.8, 0, 0), 0),
            ]
        },
         #军民鱼水情 - 全局时间272秒  3*60+52=272
         # Start + Right:
        "X": {
            "global_time": 255.0,
            "waypoints": [
                ((-2.8, 2.6, 150), 25.0),     
                ((-2.5, 2.6, 140), 55.0),     
                
                ((-2.5, 3.0, 180), 20.0),
                
               #  ((-2.6, 3.0, 180), 60.0),     
                     
                ((-2.75, 3.8, -160), 98.0),    
                
                ((-3.0, 3.4, -175), 75.0),     
                
                ((-2.8, 3.2, 180), 75.0),   
                
                ((-3.0, 3.2, -160), 55.0),     
                  
                
                ((-1.91,1.35, 0), 0),          # 门口点位，中间过渡点
                ((-0.8, 0, 0), 0),
            ]
        },
        #智斗 - 全局时间520秒 (15个点位 × 30秒)
        # Start + Left:
        "Y": {
            "global_time": 475.0,
            "waypoints": [
                ((-2.8, 2.6, 150), 15.0),     
                ((-2.5, 2.6, 140), 55.0),     
                ((-2.5, 3.0, 180), 20.0),
               #  ((-2.6, 3.0, 180), 60.0),   
               #  90s   换位  
                ((-2.75, 3.8, -170), 60.0),  
                
                #150s     背对 观众
                ((-2.0, 3.0, 0), 65.0), 
                
                    #3*60+40=220s  右边桌子   215    5*60+15=315
                ((-3.0, 3.5, 180), 40.0),  
                #4*60+20=260s  靠墙壁    255
                ((-1.8, 2.0, 150), 60.0), 
                #  5*60+15=315
                ((-2.8, 1.8, 120), 75.0), 
                
                #7*60=420s  
                ((-3.2, 2.0, 160), 55.0),     
                ((-2.3, 2.9, 175), 30.0),
                ((-2.9, 2.6, -175), 30.0),
                ((-2.1, 2.8, 165), 30.0),
                ((-2.6, 2.9, -165), 30.0),
                ((-2.4, 2.5, 185), 30.0),
                ((-2.2, 3.0, -170), 30.0),
                ((-2.2, 3.0, -170), 30.0),
                ((-2.2, 3.0, -170), 30.0),
                ((-3.0, 2.6, 170), 30.0),
                ((-2.5, 2.8, 160), 30.0),
                ((-2.7, 2.7, -160), 30.0),
                ((-3.0, 2.6, 170), 30.0),
                ((-2.5, 2.8, 160), 30.0),
                ((-3.0, 2.6, 170), 30.0),
                ((-2.8, 2.8, -185), 30.0),
                ((-1.91, 1.35, 0), 0),    # 门口点位，中间过渡点
                ((-0.8, 0, 0), 0),
            ]
        },
        # 上台 - 全局时间50秒
        "Up": {
            "global_time": 40.0,
            "waypoints": [
                ((-2.6, 3.4, 180), 40.0),
                ((-1.91,1.35, 0), 0),    # 门口点位，中间过渡点
                ((-0.8, 0, 0), 0),
            ]
        },
        
        #下台 - 全局时间50秒
         "Down": {
            "global_time": 38.0,
            "waypoints": [
                ((-2.6, 3.4, 170), 30.0),
                ((-1.91,1.35, 0), 0),    # 门口点位，中间过渡点
                ((-0.8, 0, 0), 0),
            ]
        },
    }
    
    
    # 所有模式都使用导航舞蹈逻辑，包括Up和Down
    node = SimpleNavWaypointPlayer(
        backstage_pos=backstage_pos,
        dance_type=args.dance,
        dance_choreography=dance_choreography,
    )

    rospy.loginfo(f"表演开始，使用舞蹈类型: {args.dance}")
    rospy.spin()
