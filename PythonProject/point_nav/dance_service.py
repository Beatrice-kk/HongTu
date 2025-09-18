#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import time
import numpy as np
import os
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String

# 添加G1客户端路径
sys.path.append("/home/unitree/unitree_sdk2_python/example/g1/high_level")

# 导入必要的模块
from g1_client_cwk import (
    G1ActionPlayer, 
    ChannelFactoryInitialize, 
    ChannelSubscriber, 
    LowState_,
    G1JointIndex
)

class DanceService:
    def __init__(self):
        """初始化舞蹈服务"""
        # 初始化ROS节点
        rospy.init_node("dance_service")
        
        try:
            # 初始化网络通信
            network_interface = "eth0"
            rospy.loginfo(f"正在初始化通信接口: {network_interface}")
            ChannelFactoryInitialize(0, network_interface)
            rospy.loginfo("✅ 通信初始化成功")
            
            # 创建动作播放器
            action_dir = "/home/unitree/unitree_sdk2_python/example/g1/high_level/action"
            self.player = G1ActionPlayer(action_dir)
            rospy.loginfo("✅ 动作播放器初始化成功")
            
            # 设置状态订阅器
            self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
            self.subscriber.Init(self._lowstate_callback, 10)
            rospy.loginfo("✅ 状态订阅器初始化成功")
            
            # 初始化状态
            self.initialized = False
            self.current_pose = np.zeros(15, dtype=np.float32)
            
            # 等待初始化完成
            rospy.loginfo("⏳ 等待接收机器人状态反馈...")
            wait_start = time.time()
            while not self.initialized and (time.time() - wait_start) < 10.0 and not rospy.is_shutdown():
                time.sleep(0.1)
            
            if not self.initialized and not rospy.is_shutdown():
                rospy.logwarn("⚠️ 等待状态反馈超时，使用默认姿态继续")
                self.player.current_pose = np.zeros(15, dtype=np.float32)
                self.initialized = True
            elif not rospy.is_shutdown():
                rospy.loginfo("✅ 成功接收到机器人状态反馈")
                
            # 执行初始化到安全位置
            if self.initialized and not rospy.is_shutdown():
                rospy.loginfo("🔄 执行初始化到安全位置...")
                self.player.init_to_zero_position()
                rospy.loginfo("✅ 初始化到安全位置完成")
            
            # 创建服务和订阅器
            self.dance_direction = "A"  # 默认舞蹈方向
            
            # 若已有 play_dance 服务在运行（例如由 g1_client_cwk.py 提供），则避免重复注册
            try:
                rospy.wait_for_service('play_dance', timeout=0.5)
                rospy.logwarn("检测到已有 play_dance 服务在运行，本节点不再重复提供，直接退出。")
                rospy.signal_shutdown("duplicate play_dance service detected")
                return
            except rospy.ROSException:
                pass

            # 创建舞蹈播放服务
            self.play_dance_service = rospy.Service(
                'play_dance', 
                Trigger,
                self.handle_play_dance
            )
            
            # 创建舞蹈方向订阅器
            self.direction_sub = rospy.Subscriber(
                'dance_direction', 
                String, 
                self.direction_callback
            )
            
            rospy.loginfo("🎉 舞蹈服务启动成功!")
            rospy.loginfo(f"📋 可用舞蹈动作: {list(self.player.actions.keys())}")
            
        except Exception as e:
            rospy.logerr(f"❌ 舞蹈服务初始化失败: {e}")
            rospy.signal_shutdown("初始化失败")
    
    def _lowstate_callback(self, msg):
        """处理来自机器人的状态反馈"""
        try:
            # 限制回调频率
            current_time = time.time()
            if not hasattr(self, '_last_callback_time'):
                self._last_callback_time = 0
            
            if current_time - self._last_callback_time < 0.05:  # 50ms
                return
            
            self._last_callback_time = current_time
            
            # 解析电机状态
            motor_states = msg.motor_state
            
            # 更新腰部位置
            self.current_pose[0] = motor_states[G1JointIndex.WaistYaw].q
            
            # 更新左臂位置
            left_indices = [15, 16, 17, 18, 19, 20, 21]
            for j, idx in enumerate(left_indices):
                self.current_pose[1+j] = motor_states[idx].q
                
            # 更新右臂位置
            right_indices = [22, 23, 24, 25, 26, 27, 28]
            for j, idx in enumerate(right_indices):
                self.current_pose[8+j] = motor_states[idx].q
            
            # 更新播放器的当前姿态
            self.player.current_pose = self.current_pose
            
            # 标记为已初始化
            if not self.initialized:
                self.initialized = True
                rospy.loginfo("✅ 首次接收到状态反馈")
                
            # 更新播放器状态
            self.player.update()
            
        except Exception as e:
            rospy.logerr(f"❌ 状态回调处理错误: {e}")
    
    def direction_callback(self, msg):
        """更新舞蹈方向"""
        direction = msg.data
        if direction in ['Up', 'Down', 'Left', 'Right', 'A', 'B', 'X', 'Y']:
            self.dance_direction = direction
            rospy.loginfo(f"🎯 舞蹈方向设置为: {direction}")
        else:
            rospy.logwarn(f"⚠️ 无效的舞蹈方向: {direction}")
    
    def handle_play_dance(self, req):
        """处理播放舞蹈的服务请求"""
        response = TriggerResponse()
        
        try:
            rospy.loginfo(f"🎭 收到舞蹈播放请求: {self.dance_direction}")
            
            # 检查舞蹈是否存在
            if self.dance_direction not in self.player.actions:
                available_dances = list(self.player.actions.keys())
                rospy.logwarn(f"❌ 舞蹈 '{self.dance_direction}' 不存在！可用舞蹈: {available_dances}")
                response.success = False
                response.message = f"Dance '{self.dance_direction}' does not exist. Available: {available_dances}"
                return response
            
            # 检查播放器状态
            rospy.loginfo(f" 当前播放器状态: {self.player.state}")
            
            # 如果正在播放其他动作，先停止
            if self.player.state != "stopped":
                rospy.loginfo("⏹️ 停止当前播放的动作...")
                self.player.stop_play()
                # 等待停止完成
                wait_start = time.time()
                while self.player.state != "stopped" and (time.time() - wait_start) < 5.0:
                    time.sleep(0.1)
            
            # 开始播放舞蹈
            rospy.loginfo(f"▶️ 开始播放舞蹈: {self.dance_direction}")
            result = self.player.play_action(self.dance_direction, speed=1.0)
            
            if result:
                # 等待播放完成
                rospy.loginfo("⏳ 等待舞蹈播放完成...")
                start_time = time.time()
                max_wait_time = 120.0  # 最多等待2分钟
                
                while (self.player.state != "stopped" and 
                       (time.time() - start_time) < max_wait_time and 
                       not rospy.is_shutdown()):
                    time.sleep(0.1)
                
                if self.player.state == "stopped":
                    rospy.loginfo("✅ 舞蹈播放完成")
                    response.success = True
                    response.message = f"Successfully played dance: {self.dance_direction}"
                else:
                    rospy.logwarn("⚠️ 舞蹈播放超时")
                    response.success = False
                    response.message = f"Dance playback timeout: {self.dance_direction}"
            else:
                rospy.logerr(f"❌ 舞蹈播放启动失败: {self.dance_direction}")
                response.success = False
                response.message = f"Failed to start dance: {self.dance_direction}"
                
        except Exception as e:
            rospy.logerr(f"❌ 舞蹈播放过程中出错: {str(e)}")
            response.success = False
            response.message = f"Error during dance playback: {str(e)}"
            
            # 尝试停止播放
            try:
                self.player.stop_play()
            except:
                pass
        
        return response
    
    def list_available_dances(self):
        """列出所有可用的舞蹈"""
        dances = []
        for direction, action in self.player.actions.items():
            dances.append({
                "direction": direction,
                "name": action['name'],
                "frames": len(action['data']),
                "duration": len(action['data']) / action['fps']
            })
        return dances

if __name__ == "__main__":
    try:
        service = DanceService()
        rospy.loginfo(" 舞蹈服务正在运行...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo(" 舞蹈服务被中断")
    except Exception as e:
        rospy.logerr(f"❌ 舞蹈服务运行错误: {e}")
    finally:
        rospy.loginfo("🔚 舞蹈服务已退出")