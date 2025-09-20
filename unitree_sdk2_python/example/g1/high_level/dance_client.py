#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import numpy as np
import glob
import threading
from datetime import datetime
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String as RosString

# 添加SDK路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../../"))
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

# -------------------------------
# G1 关节索引
# -------------------------------
class G1JointIndex:
    WaistYaw = 12
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28
    kArmSdkEnable = 29

# -------------------------------
# 动作播放器（精简版）
# -------------------------------
class G1DancePlayer:
    def __init__(self, action_dir=None):
        """
        初始化G1动作播放器
        
        Args:
            action_dir: 动作文件目录路径
        """
        # 保存动作目录路径，如果未提供则使用默认相对路径
        if action_dir is None:
            # 使用相对于当前文件的路径
            self.action_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "action")
        else:
            # 展开用户目录符号
            self.action_dir = os.path.expanduser(action_dir)
        
        # 初始化动作字典
        self.actions = {}
        
        # 初始化状态
        self.state = "stopped"
        self.current_action = None
        self.fps = 30.0
        self.dt = 0.0333
        self.publisher = None
        self.low_cmd = None
        self.crc = CRC()
        self.loop = False

        # 状态与时间
        self.ramp_in_duration = 0.8        # 平滑进入时间
        self.move_to_initial_duration = 2.5  # 回到初始姿态的时间
        self.ramp_start_time = None
        self.start_time = None
        self.current_frame = 0

        # 当前反馈 (初始化为None，表示尚未获取到反馈)
        self.current_pose = None
        
        # 初始姿态（固定为零位）
        self.initial_pose = np.zeros(15, dtype=np.float32)
        
        # 程序启动时的姿态
        self.startup_pose = None

        # 控制参数
        self.base_kp_waist = 65.0
        self.base_kp_arm = 40.0
        self.base_kd_waist = 7
        self.base_kd_arm = 5
        
        # 动作幅度缩放因子
        self.action_scale_factor = 0.9
        
        # 最大关节速度限制
        self.max_joint_velocity = 1.0
        
        # 平滑插值参数
        self.smoothing_factor = 0.15
        
        # 初始化标志
        self.has_sent_stop_cmd = False
        
        # 加载动作并设置发布器
        self.load_actions()
        self.setup_publisher()
        
        # 状态同步
        self.state_flags = {'initial_pose_received': False, 'initialization_done': False}
        
        # 创建状态订阅器
        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self.lowstate_callback, 10)
        
        print(f"✅ G1DancePlayer 初始化完成，加载了 {len(self.actions)} 个动作")
    
    def load_actions(self):
        """加载所有动作文件，支持加载分割的动作文件"""
        print(f"📁 加载动作文件从目录: {self.action_dir}")
        
        # 定义方向映射 - 将遥控器方向映射到目录名
        direction_map = {
            'up': 'Up',
            'down': 'Down', 
            'left': 'Left',
            'right': 'Right',
            'a': 'A',
            'b': 'B',
            'x': 'X',
            'y': 'Y'
        }
        
        # 遍历所有方向目录
        for direction_dir in os.listdir(self.action_dir):
            direction_path = os.path.join(self.action_dir, direction_dir)
            if os.path.isdir(direction_path) and direction_dir.lower() in direction_map:
                direction_key = direction_map[direction_dir.lower()]
                
                # 收集所有npz文件并按名称分组
                npz_files = glob.glob(os.path.join(direction_path, "*.npz"))
                action_groups = {}
                
                # 按基本名称分组（去除数字后缀）
                for npz_file in npz_files:
                    base_name = os.path.basename(npz_file).replace(".npz", "")
                    # 移除数字后缀（如"智斗1" -> "智斗"）
                    clean_name = ''.join([c for c in base_name if not c.isdigit()])
                    if clean_name not in action_groups:
                        action_groups[clean_name] = []
                    action_groups[clean_name].append(npz_file)
                
                # 对每个组内的文件按数字排序
                for clean_name in action_groups:
                    action_groups[clean_name].sort(key=lambda x: self._get_file_number(x))
                
                # 查找同名的音频文件
                audio_files = glob.glob(os.path.join(direction_path, "*.wav"))
                audio_map = {os.path.splitext(os.path.basename(f))[0]: f for f in audio_files}
                
                # 处理每个动作组
                for clean_name, npz_file_group in action_groups.items():
                    try:
                        # 加载并合并所有分割的动作文件
                        all_action_data = []
                        total_frames = 0
                        fps = 30.0
                        
                        # 按顺序加载所有分割的动作文件
                        for npz_file in npz_file_group:
                            data = np.load(npz_file)
                            if 'qpos' not in data:
                                print(f"⚠️  动作文件 {npz_file} 缺少 'qpos' 字段，跳过")
                                continue
                                
                            qpos = data['qpos']
                            
                            # 支持 (T, 9) 自动扩展为 (T, 15)
                            if qpos.ndim == 2 and qpos.shape[1] == 9:
                                print(f"⚠️  检测到 (T, 9) 格式，自动补 0 扩展为 (T, 15)")
                                zeros = np.zeros((qpos.shape[0], 6), dtype=qpos.dtype)
                                qpos = np.hstack([qpos, zeros])
                            elif qpos.ndim != 2 or qpos.shape[1] != 15:
                                print(f"⚠️  动作文件 {npz_file} 格式不正确，跳过")
                                continue
                                
                            action_data = qpos.astype(np.float32)
                            all_action_data.append(action_data)
                            total_frames += len(action_data)
                            
                            # 获取fps（使用第一个文件的fps）
                            if 'fps' in data:
                                raw_fps = data['fps']
                                fps = float(raw_fps.item() if isinstance(raw_fps, np.ndarray) else raw_fps)
                        
                        if not all_action_data:
                            continue
                            
                        # 合并所有动作数据
                        merged_action_data = np.vstack(all_action_data)
                        
                        # 对合并后的动作序列进行平滑处理
                        print(f"🔧 对动作序列进行预处理: {clean_name} (共{len(npz_file_group)}个文件)")
                        max_angle_delta = 0.25  # 可根据动作类型调整
                        merged_action_data = self._smooth_action_sequence(merged_action_data, max_angle_delta=max_angle_delta, min_fps=fps)
                        
                        # 查找对应的音频文件
                        audio_file = None
                        if clean_name in audio_map:
                            audio_file = audio_map[clean_name]
                        elif len(npz_file_group) > 0:
                            base_name = os.path.basename(npz_file_group[0]).replace(".npz", "")
                            # 移除数字后缀查找音频文件
                            clean_base_name = ''.join([c for c in base_name if not c.isdigit()])
                            if clean_base_name in audio_map:
                                audio_file = audio_map[clean_base_name]
                        
                        # 保存动作
                        self.actions[direction_key] = {
                            'file': npz_file_group[0],  # 使用第一个文件作为代表
                            'data': merged_action_data,
                            'fps': fps,
                            'dt': 1.0 / fps,
                            'name': clean_name,
                            'source_dir': direction_dir,  # 保存源目录名，用于调试
                            'audio_file': audio_file,  # 保存对应的音频文件路径
                            'original_files': npz_file_group  # 保存原始文件列表
                        }
                        
                        print(f"✅ 加载动作: {direction_key} (来自 {direction_dir}/) -> '{clean_name}' | 帧数: {len(merged_action_data)} | fps: {fps:.1f} | 文件数: {len(npz_file_group)}")
                        if audio_file:
                            print(f"🎵 关联音频文件: {audio_file}")
                    except Exception as e:
                        print(f"❌ 加载动作文件组 {clean_name} 失败: {e}")
        
        if not self.actions:
            print("⚠️  未找到任何动作文件")
        else:
            print(f"✅ 成功加载 {len(self.actions)} 个动作")
    
    def _get_file_number(self, filepath):
        """从文件路径中提取数字后缀"""
        filename = os.path.basename(filepath).replace(".npz", "")
        numbers = ''.join([c for c in filename if c.isdigit()])
        return int(numbers) if numbers else 0
    
    def _smooth_action_sequence(self, action_data, max_angle_delta=0.25, min_fps=30.0):
        """对动作序列进行平滑处理，在关节角度变化剧烈的地方插入中间帧"""
        if len(action_data) < 2:
            return action_data
            
        smoothed_data = []
        
        # 检查是否为剧烈动作
        overall_motion = np.sum(np.abs(action_data[-1] - action_data[0]))
        is_intense_action = overall_motion > 2.0  # 如果整体运动幅度大于2弧度，认为是剧烈动作
        
        # 对于剧烈动作，使用更小的角度变化阈值
        if is_intense_action:
            max_angle_delta = 0.15  # 降低阈值以增加插帧密度
            print(f"⚔️  检测到剧烈动作，使用更高精度插帧处理 (阈值: {max_angle_delta} rad)")
        
        # 遍历相邻帧
        for i in range(len(action_data) - 1):
            current_frame = action_data[i]
            next_frame = action_data[i + 1]
            
            # 计算每关节角度变化
            delta = np.abs(next_frame - current_frame)
            max_delta = np.max(delta)
            
            # 添加当前帧
            smoothed_data.append(current_frame)
            
            # 如果最大角度变化超过阈值，则进行插帧
            if max_delta > max_angle_delta:
                # 计算需要插入的帧数
                base_frames = max_angle_delta / 0.15  # 基础帧数
                dynamic_frames = max_delta / max_angle_delta  # 根据实际变化调整
                num_insert_frames = int(np.ceil(dynamic_frames * base_frames))
                num_insert_frames = max(1, min(num_insert_frames, 30))  # 最多插入30帧
                
                # 插入中间帧
                for j in range(1, num_insert_frames + 1):
                    t = j / (num_insert_frames + 1)
                    # 使用平滑插值函数
                    smooth_t = t * t * (3 - 2 * t)  # 平滑步进插值
                    interpolated_frame = (1 - smooth_t) * current_frame + smooth_t * next_frame
                    smoothed_data.append(interpolated_frame)
        
        # 添加最后一帧
        smoothed_data.append(action_data[-1])
        
        result = np.array(smoothed_data, dtype=np.float32)
        if len(result) > len(action_data):
            print(f"📈 动作序列平滑处理完成: {len(action_data)} 帧 -> {len(result)} 帧 (增加 {len(result) - len(action_data)} 帧)")
        else:
            print(f"✅ 动作序列检查完成: {len(action_data)} 帧 (无需插帧)")
        
        return result
    
    def setup_publisher(self):
        """设置低级命令发布器"""
        self.publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.publisher.Init()
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
    
    def play_action(self, direction, speed=1.0):
        """播放指定方向的动作
        
        Args:
            direction: 动作方向键名称 ('Up', 'Down', 'Left', 'Right', 'A', 'B', 'X', 'Y')
            speed: 播放速度倍数，默认为1.0（正常速度）
                  大于1.0表示加速播放，小于1.0表示减速播放
                  
        Returns:
            bool: 成功启动播放返回True，否则返回False
        """
        print(f"🎮 尝试播放 {direction} 方向的动作，速度: {speed}x")
        
        if direction not in self.actions:
            print(f"⚠️  未找到 {direction} 方向的动作")
            # 显示已加载的动作
            print("📋 当前已加载的动作:")
            for dir_key, action in self.actions.items():
                print(f"    {dir_key}: {action['name']}")
            return False
            
        if self.state not in ["stopped"]:
            print(f"⚠️  当前正在播放动作，无法播放新动作")
            return False
            
        action = self.actions[direction]
        self.current_action = action
        
        # 根据速度调整fps和dt
        original_fps = action['fps']
        adjusted_fps = original_fps * speed
        adjusted_dt = 1.0 / adjusted_fps
        
        self.fps = adjusted_fps
        self.dt = adjusted_dt
        self.action_data = action['data']
        
        print(f"▶️ 开始播放动作: {direction} | 名称: '{action['name']}' | 帧数: {len(self.action_data)} | 原始fps: {original_fps:.1f} | 调整后fps: {adjusted_fps:.1f}")
        
        self.state = "ramp_in"  # 平滑进入状态
        self.ramp_start_time = time.time()
        self.start_time = self.ramp_start_time
        self.current_frame = 0
        
        try:
            self.low_cmd.motor_cmd[G1JointIndex.kArmSdkEnable].q = 1.0
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.publisher.Write(self.low_cmd)
        except Exception as e:
            print(f"❌ 发送播放命令失败: {e}")
            return False
            
        return True
    
    def stop_play(self):
        """停止当前动作播放并回到初始姿态"""
        # 只有在播放状态才进入回到初始姿态的流程
        if self.state in ["playing", "ramp_in"]:
            print(f"⏹️ 动作结束，进入平滑回到初始姿态流程")
            
            self.state = "move_to_initial"
            self.ramp_start_time = time.time()
            # 记录当前实际位置作为过渡的起点
            if self.current_pose is not None:
                self.transition_start_pose = self.current_pose.copy()
            else:
                # 如果没有当前位置反馈，使用初始姿态
                self.transition_start_pose = self.startup_pose if hasattr(self, 'startup_pose') and self.startup_pose is not None else self.initial_pose
            return True
        elif self.state == "move_to_initial":
            # 如果已经在回到初始姿态的过程中，直接完成
            self.state = "stopped"
            print("✅ 状态已设置为 stopped")
            return True
        return False
    
    def init_to_zero_position(self):
        """程序启动后主动执行平滑移动到预设安全位置"""
        print("🔄 程序启动，开始执行初始化到预设安全位置...")
        
        # 等待获取实际的当前位置反馈
        wait_start = time.time()
        while self.current_pose is None and (time.time() - wait_start) < 5.0:  # 等待最多5秒
            print("⏳ 等待接收实际关节位置反馈...")
            time.sleep(0.1)
            
        if self.current_pose is None:
            # 如果超时仍未收到反馈，不能继续执行初始化
            print("❌ 超时未收到反馈，无法执行初始化")
            return
            
        print(f"✅ 收到实际关节位置反馈: {self.current_pose[:3]}")
        
        # 定义预设的安全初始姿态
        target_pose = np.zeros(15, dtype=np.float32)
        # 左臂预设位置
        target_pose[1:8] = [0.292, 0.220, -0.011, 0.984, 0.097, 0.022, -0.036]
        # 右臂预设位置
        target_pose[8:15] = [0.290, -0.219, 0.025, 0.974, -0.088, 0.032, 0.021]
        
        start_pose = self.current_pose.copy()
        
        print(f"📍 起始位置: {start_pose[:3]}")
        print(f"📍 目标位置: {target_pose[:3]}")
        
        # 检查是否已经在目标位置附近，如果是则跳过平滑过渡
        position_diff = np.linalg.norm(start_pose - target_pose)
        print(f"📏 起始位置与目标位置差异: {position_diff:.3f}")
        if position_diff < 0.05:  # 如果差异小于0.05弧度，则认为已经在位置上
            print("✅ 已经在目标位置附近，跳过平滑过渡")
            self.startup_pose = target_pose.copy()
            # 发送一次位置命令确保位置稳定
            self._send_pose(target_pose, dq=np.zeros(15), kp_scale=0.3, kd_scale=1.0)
            time.sleep(0.1)
            # 初始化完成后设置状态为stopped，避免持续发送指令
            self.state = "stopped"
            return
            
        # 平滑过渡到目标位置
        print("🔄 开始平滑过渡到预设安全位置...")
        duration = 3.0  # 过渡时间
        start_time = time.time()
        
        while True:
            elapsed = time.time() - start_time
            if elapsed >= duration:
                break
                
            # 计算插值比例
            ratio = elapsed / duration
            smooth_ratio = (1 - np.cos(ratio * np.pi)) / 2  # cosine插值
            
            # 计算当前目标位置
            current_target = (1 - smooth_ratio) * start_pose + smooth_ratio * target_pose
            
            # 发送命令，使用较低的刚度确保安全
            self._send_pose(current_target, dq=np.zeros(15), kp_scale=0.3, kd_scale=1.0)
            time.sleep(0.02)  # 50Hz控制频率
            
        # 确保最终位置
        self._send_pose(target_pose, dq=np.zeros(15), kp_scale=0.3, kd_scale=1.0)
        time.sleep(0.1)
        
        # 保存这个预设位置作为初始姿态
        self.startup_pose = target_pose.copy()
        print("✅ 初始化到预设安全位置完成")
        print(f"📍 当前位置: {target_pose[:3]}")
        
        # 初始化完成后设置状态为stopped，避免持续发送指令
        self.state = "stopped"
    
    def _send_pose(self, q, dq=None, kp_scale=1.0, kd_scale=1.0):
        """发送关节角度位置命令"""
        # 限制发送频率以减少CPU使用
        current_time = time.time()
        if not hasattr(self, '_last_send_time'):
            self._last_send_time = 0
            
        # 在动作播放期间提高发送频率到25ms一次 (~40Hz)，其他时候保持40ms
        if self.state in ["ramp_in", "playing"]:
            send_interval = 0.025  # 动作播放期间40Hz
        else:
            send_interval = 0.04   # 其他时候25Hz
            
        if current_time - self._last_send_time < send_interval:
            return
            
        self._last_send_time = current_time
        
        if self.low_cmd is None:
            self.low_cmd = unitree_hg_msg_dds__LowCmd_()
            
        cmd = self.low_cmd
        if dq is None:
            dq = np.zeros(15)

        # 根据状态调整控制参数
        if self.state in ["ramp_in", "playing"]:
            kp_waist = self.base_kp_waist * kp_scale * 0.7
            kp_arm = self.base_kp_arm * kp_scale * 0.6
            kd_waist = self.base_kd_waist * kd_scale * 1.5
            kd_arm = self.base_kd_arm * kd_scale * 1.8
        else:
            kp_waist = self.base_kp_waist * kp_scale * 0.5
            kp_arm = self.base_kp_arm * kp_scale * 0.4
            kd_waist = self.base_kd_waist * kd_scale * 1.8
            kd_arm = self.base_kd_arm * kd_scale * 2.2

        # 腰部
        c = cmd.motor_cmd[G1JointIndex.WaistYaw]
        c.mode = 10
        c.q = float(q[0])
        c.dq = float(dq[0])
        c.tau = 0
        c.kp = kp_waist
        c.kd = kd_waist

        # 左臂
        left_indices = [15,16,17,18,19,20,21]
        for j, idx in enumerate(left_indices):
            c = cmd.motor_cmd[idx]
            c.mode = 10
            c.q = float(q[1+j])
            c.dq = float(dq[1+j])
            c.tau = 0
            c.kp = kp_arm
            c.kd = kd_arm

        # 右臂
        right_indices = [22,23,24,25,26,27,28]
        for j, idx in enumerate(right_indices):
            c = cmd.motor_cmd[idx]
            c.mode = 10
            c.q = float(q[8+j])
            c.dq = float(dq[8+j])
            c.tau = 0
            c.kp = kp_arm
            c.kd = kd_arm

        cmd.crc = self.crc.Crc(cmd)
        if self.publisher is not None:
            try:
                self.publisher.Write(cmd)
            except Exception as e:
                print(f"⚠️ 发送命令失败: {e}")
    
    def _send_interpolated_frame(self, smooth_ratio, target_idx):
        """发送插值帧，用于平滑过渡"""
        target_q = self.action_data[target_idx].copy()
        start_q = self.current_pose if self.current_pose is not None else np.zeros(15)
        # 应用动作幅度缩放因子
        scaled_diff = (target_q - start_q) * self.action_scale_factor
        end_q = start_q + scaled_diff
        
        # 使用分段策略的高阶插值函数提升平滑性
        if smooth_ratio < 0.5:
            # 前半段使用平方函数，缓慢启动
            smooth_ratio = smooth_ratio * smooth_ratio
        else:
            # 后半段使用高阶函数，提供更好的加速度控制
            smooth_ratio = smooth_ratio * smooth_ratio * (3 - 2 * smooth_ratio)
            
        # 添加额外的平滑处理
        if not hasattr(self, 'ramp_smoothed_pose') or self.ramp_smoothed_pose is None:
            self.ramp_smoothed_pose = start_q.copy()
        
        # 计算目标插值位置
        target_interp_q = (1 - smooth_ratio) * start_q + smooth_ratio * end_q
        
        # 应用额外的平滑处理
        smoothing_factor_ramp = 0.2  # 平滑系数
        self.ramp_smoothed_pose = self.ramp_smoothed_pose + smoothing_factor_ramp * (target_interp_q - self.ramp_smoothed_pose)
        interp_q = self.ramp_smoothed_pose.copy()
        
        # 添加关节速度限制以减少抖动
        if hasattr(self, '_last_sent_pose') and self._last_sent_pose is not None:
            # 计算理论上的关节速度
            dt = 0.025  # 与发送频率匹配
            theoretical_velocity = (interp_q - self._last_sent_pose) / dt
            
            # 限制关节速度
            velocity_limited_q = np.zeros_like(interp_q)
            for i in range(len(interp_q)):
                max_vel = self.max_joint_velocity
                actual_vel = theoretical_velocity[i]
                if abs(actual_vel) > max_vel:
                    # 限制速度
                    velocity_limited_q[i] = self._last_sent_pose[i] + np.sign(actual_vel) * max_vel * dt
                else:
                    velocity_limited_q[i] = interp_q[i]
            
            interp_q = velocity_limited_q
        
        # 保存当前发送的姿态用于下次速度计算
        self._last_sent_pose = interp_q.copy()
        
        # 优化控制参数，在平滑度和力度之间取得平衡
        kp_scale = 0.5 + 0.1 * smooth_ratio
        kd_scale = 1.2 + 0.3 * smooth_ratio
        self._send_pose(interp_q, dq=np.zeros(15), kp_scale=kp_scale, kd_scale=kd_scale)

    def _send_frame(self, frame_idx):
        """发送特定帧的动作数据"""
        q = self.action_data[frame_idx]
        # 使用适度的平滑函数来调整控制参数
        progress = frame_idx / len(self.action_data) if len(self.action_data) > 0 else 0
        smooth_factor = progress * progress * (3 - 2 * progress)
        
        # 根据播放进度调整控制参数
        kp_scale = 0.8 + 0.2 * smooth_factor
        kd_scale = 0.9 + 0.1 * smooth_factor
        
        self._send_pose(q, dq=np.zeros(15), kp_scale=kp_scale, kd_scale=kd_scale)
    
    def update(self):
        """更新动作状态和控制"""
        # 限制update函数的执行频率
        current_time = time.time()
        
        # 使用实例属性来存储上次调用时间
        if not hasattr(self, '_last_update_call'):
            self._last_update_call = 0
            
        # 限制update调用频率为50ms一次
        if current_time - self._last_update_call < 0.05:
            return
            
        self._last_update_call = current_time
        
        # 在停止状态下不发送任何控制指令
        if self.state == "stopped":
            return
            
        # 如果尚未获取到当前位置反馈，使用零位作为默认位置
        if self.current_pose is None:
            self.current_pose = np.zeros(15, dtype=np.float32)

        # 获取当前时间
        t = time.time()

        # -------------------------------
        # 2. playing: 精确播放，播完再退出
        # -------------------------------
        if self.state == "playing":
            elapsed = t - self.start_time
            total_duration = len(self.action_data) * self.dt  # 基于动作帧数计算
            
            # 检查动作是否播放完成
            if elapsed >= total_duration:
                print(f"🎬 动作播放完毕（{len(self.action_data)} 帧），进入退出流程")
                self.stop_play()
            else:
                # 精确计算目标帧索引，确保按照原始动作数据播放
                target_frame = max(0, min(int(elapsed / self.dt), len(self.action_data) - 1))
                self.current_frame = target_frame
                self._send_frame(target_frame)
                
                # 定期报告播放进度
                if not hasattr(self, '_last_progress_report') or (t - self._last_progress_report) >= 1.0:
                    progress = elapsed / total_duration if total_duration > 0 else 0
                    print(f"🎵 播放进度: {progress:.1%} ({elapsed:.1f}/{total_duration:.1f}s)")
                    self._last_progress_report = t
            return

        # -------------------------------
        # 1. ramp_in: 当前 → 第一帧（cosine）
        # -------------------------------
        if self.state == "ramp_in":
            elapsed = t - self.ramp_start_time
            ratio = min(elapsed / self.ramp_in_duration, 1.0)
            # 使用标准平滑步进插值
            self._send_interpolated_frame(ratio, target_idx=0)
            if ratio >= 1.0:
                self.state = "playing"
                # 重置start_time，确保从第一帧开始播放
                self.start_time = t
                self.current_frame = 0
                # 清除ramp阶段的平滑姿态缓存
                if hasattr(self, 'ramp_smoothed_pose'):
                    delattr(self, 'ramp_smoothed_pose')
            return

        # -------------------------------
        # 3. move_to_initial: 回到初始姿态
        # -------------------------------
        if self.state == "move_to_initial":
            elapsed = t - self.ramp_start_time
            duration = self.move_to_initial_duration
            ratio = min(elapsed / duration, 1.0)
            
            # 使用更高阶的插值函数提升平滑性
            smooth_ratio = ratio * ratio * ratio * (10 - 15 * ratio + 6 * ratio * ratio)
            
            # 从当前实际位置开始，而不是从动作的最后一帧开始
            start_q = self.current_pose if self.current_pose is not None else self.action_data[-1]
            # 回到程序启动时保存的初始姿态
            target_q = self.startup_pose if hasattr(self, 'startup_pose') and self.startup_pose is not None else self.initial_pose
            
            # 进行插值计算
            interp_q = (1 - smooth_ratio) * start_q + smooth_ratio * target_q
            
            # 添加关节速度限制以减少抖动
            if hasattr(self, '_last_sent_pose') and self._last_sent_pose is not None:
                # 计算理论上的关节速度
                dt = 0.04  # 与发送频率匹配
                theoretical_velocity = (interp_q - self._last_sent_pose) / dt
                
                # 限制关节速度
                velocity_limited_q = np.zeros_like(interp_q)
                for i in range(len(interp_q)):
                    max_vel = self.max_joint_velocity * 0.7  # 回到初始姿态时使用更低的速度限制
                    actual_vel = theoretical_velocity[i]
                    if abs(actual_vel) > max_vel:
                        # 限制速度
                        velocity_limited_q[i] = self._last_sent_pose[i] + np.sign(actual_vel) * max_vel * dt
                    else:
                        velocity_limited_q[i] = interp_q[i]
                
                interp_q = velocity_limited_q
            
            # 保存当前发送的姿态用于下次速度计算
            self._last_sent_pose = interp_q.copy()
            
            # 发送插值位置命令
            self._send_pose(interp_q, dq=np.zeros(15), kp_scale=0.2, kd_scale=1.5)

            # 如果插值完成
            if ratio >= 1.0:
                print("🎯 已回到初始姿态")
                print(f"📍 最终位置: {interp_q[:3]}")
                # 发送最终位置命令
                self._send_pose(target_q, dq=np.zeros(15), kp_scale=0.2, kd_scale=1.2)
                # 设置状态为停止
                self.state = "stopped"
                print("✅ 状态已设置为 stopped")
                
                # 清空当前动作
                self.current_action = None
                # 清除平滑姿态缓存
                if hasattr(self, 'smoothed_pose'):
                    delattr(self, 'smoothed_pose')
                # 清除ramp阶段的平滑姿态缓存
                if hasattr(self, 'ramp_smoothed_pose'):
                    delattr(self, 'ramp_smoothed_pose')
            return
    
    def lowstate_callback(self, msg):
        """接收G1机器人状态反馈的回调函数"""
        try:
            # 限制回调函数的处理频率
            current_time = time.time()
            if not hasattr(self, '_last_callback_time'):
                self._last_callback_time = 0
            
            # 限制回调处理频率为50ms一次
            if current_time - self._last_callback_time < 0.05:
                return
            
            self._last_callback_time = current_time
            
            # 提取关节状态
            motor_states = msg.motor_state
            q_feedback = np.zeros(15, dtype=np.float32)
            q_feedback[0] = motor_states[G1JointIndex.WaistYaw].q
            left_indices = [15,16,17,18,19,20,21]
            for j, idx in enumerate(left_indices):
                q_feedback[1+j] = motor_states[idx].q
            right_indices = [22,23,24,25,26,27,28]
            for j, idx in enumerate(right_indices):
                q_feedback[8+j] = motor_states[idx].q
            
            # 更新当前位置反馈
            self.current_pose = q_feedback
            
            # 首次收到位置反馈
            if not self.state_flags['initial_pose_received']:
                print("🔄 首次收到位置反馈")
                self.state_flags['initial_pose_received'] = True
                
                # 收到反馈后执行初始化到零位
                if not self.state_flags['initialization_done']:
                    print("🔄 开始初始化流程...")
                    self.init_to_zero_position()
                    print("✅ 初始化流程完成")
                    self.state_flags['initialization_done'] = True
                    print("✅ 系统就绪！")
            
        except Exception as e:
            print(f"❌ 回调处理失败: {e}")
    
    def get_available_actions(self):
        """获取可用动作列表"""
        return list(self.actions.keys())
    
    def is_ready(self):
        """检查动作播放器是否已准备好"""
        return self.state_flags['initialization_done']
    
    def is_playing(self):
        """检查是否正在播放动作"""
        return self.state in ["playing", "ramp_in"]
    
    def wait_until_ready(self, timeout=10.0):
        """等待直到动作播放器准备好"""
        start_time = time.time()
        while not self.is_ready() and (time.time() - start_time) < timeout:
            time.sleep(0.1)
        return self.is_ready()

# -------------------------------
# ROS 服务接口
# -------------------------------
class G1DanceNode:
    def __init__(self, node_name="g1_dance_service", action_dir=None):
        """
        初始化G1舞蹈ROS节点
        
        Args:
            node_name: ROS节点名称
            action_dir: 动作文件目录路径
        """
        # 初始化通信
        try:
            # 如果已经初始化过，则不再重复初始化
            if not hasattr(G1DanceNode, '_channel_initialized'):
                ChannelFactoryInitialize(0, "eth0")  # 默认使用eth0网卡
                G1DanceNode._channel_initialized = True
                print(f"✅ 通信初始化成功")
        except Exception as e:
            print(f"❌ 通信初始化失败: {e}")
            raise
        
        # 初始化ROS节点
        if not rospy.core.is_initialized():
            rospy.init_node(node_name, anonymous=True, disable_signals=True)
        
        # 当前舞蹈方向
        self.current_dance_direction = {'value': 'A'}
        
        # 初始化舞蹈播放器
        self.player = G1DancePlayer(action_dir)
        
        # 等待播放器准备好
        if not self.player.wait_until_ready(timeout=10.0):
            print("⚠️ 播放器未能在超时时间内准备好，但将继续初始化服务")
        
        # 设置ROS订阅者和服务
        self.direction_sub = rospy.Subscriber("dance_direction", RosString, self._direction_callback, queue_size=10)
        self.play_service = rospy.Service("play_dance", Trigger, self._handle_play_dance)
        self.stop_service = rospy.Service("stop_dance", Trigger, self._handle_stop_dance)
        
        print("✅ ROS 服务已提供: play_dance, stop_dance，订阅: dance_direction")
        print("💃 可用动作:", self.player.get_available_actions())
    
    def _direction_callback(self, msg: RosString):
        """处理舞蹈方向消息的回调函数"""
        try:
            val = msg.data
            if val in ['Up','Down','Left','Right','A','B','X','Y']:
                self.current_dance_direction['value'] = val
                rospy.loginfo(f"dance_direction 设置为: {val}")
            else:
                rospy.logwarn(f"无效 dance_direction: {val}")
        except Exception as e:
            rospy.logerr(f"direction 回调错误: {e}")
    
    def _handle_play_dance(self, _req):
        """处理播放舞蹈的服务请求"""
        resp = TriggerResponse()
        try:
            direction = self.current_dance_direction['value']
            rospy.loginfo(f"收到 play_dance 请求: {direction}")

            if direction not in self.player.actions:
                available = self.player.get_available_actions()
                resp.success = False
                resp.message = f"Dance '{direction}' 不存在，可用: {available}"
                return resp

            # 若正在播放，先停止
            if self.player.is_playing():
                self.player.stop_play()
                wait_t0 = time.time()
                while self.player.is_playing() and (time.time()-wait_t0) < 5.0:
                    time.sleep(0.1)

            ok = self.player.play_action(direction, speed=1.0)
            if not ok:
                resp.success = False
                resp.message = f"启动失败: {direction}"
                return resp

            # 等待完成（最多120秒）
            t0 = time.time()
            while self.player.state != "stopped" and (time.time()-t0) < 120.0:
                time.sleep(0.1)
                # 持续更新播放器状态
                self.player.update()

            if self.player.state == "stopped":
                resp.success = True
                resp.message = f"舞蹈完成: {direction}"
            else:
                resp.success = False
                resp.message = f"舞蹈超时: {direction}"
        except Exception as e:
            rospy.logerr(f"处理 play_dance 出错: {e}")
            resp.success = False
            resp.message = str(e)
        return resp
    
    def _handle_stop_dance(self, _req):
        """处理停止舞蹈的服务请求"""
        resp = TriggerResponse()
        try:
            rospy.loginfo("收到 stop_dance 请求")
            
            if self.player.is_playing():
                self.player.stop_play()
                # 等待动作停止（最多5秒）
                wait_t0 = time.time()
                while self.player.state != "stopped" and (time.time()-wait_t0) < 5.0:
                    time.sleep(0.1)
                    # 持续更新播放器状态
                    self.player.update()
                
                if self.player.state == "stopped":
                    resp.success = True
                    resp.message = "舞蹈已停止"
                else:
                    resp.success = False
                    resp.message = "停止舞蹈超时"
            else:
                resp.success = True
                resp.message = "无正在播放的舞蹈"
        except Exception as e:
            rospy.logerr(f"处理 stop_dance 出错: {e}")
            resp.success = False
            resp.message = str(e)
        return resp
    
    def run(self):
        """运行ROS节点"""
        rate = rospy.Rate(20)  # 20Hz
        try:
            print("✅ G1舞蹈节点已启动，等待请求...")
            while not rospy.is_shutdown():
                # 更新动作状态
                self.player.update()
                rate.sleep()
        except KeyboardInterrupt:
            print("\n👋 收到中断信号，准备退出")
        except Exception as e:
            print(f"\n❌ 节点运行出错: {e}")
            import traceback
            traceback.print_exc()
        finally:
            print("\n👋 节点退出")

# 主函数
def main():
    try:
        # 获取当前文件所在目录
        current_dir = os.path.dirname(os.path.abspath(__file__))
        action_dir = os.path.join(current_dir, "action")  # 默认动作目录
        
        # 创建并运行舞蹈节点
        dance_node = G1DanceNode(action_dir=action_dir)
        dance_node.run()
    except Exception as e:
        print(f"❌ 主函数出错: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()