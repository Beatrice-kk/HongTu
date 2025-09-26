#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
G1机器人动作播放器模块
"""

import os
import sys
import time
import json
import numpy as np
import math
import glob
import threading
import subprocess
from datetime import datetime

# 导入必要的SDK模块
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../../"))
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

# 导入关节索引
from g1_joint_index import G1JointIndex

# 导入新的音频处理模块
try:
    from g1_audio_processor import G1AudioProcessor
except ImportError:
    # 如果直接运行脚本，可能需要添加当前目录到路径
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from g1_audio_processor import G1AudioProcessor

class G1ActionPlayer:
    """G1机器人动作播放器"""
    
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
        
        # 预设TTS文本配置
        self.tts_presets = {

            'A': "亲爱的游客朋友们，大家好！欢迎光临国家5诶级旅游景区——沙家浜！这里不仅是一座风光秀美的江南水乡，更是一方承载着芦荡火种、鱼水情深的红色记忆的圣地。在这里，您可以走进沙家浜革命历史纪念馆，聆听那段可歌可泣的英雄故事，感受新四军与人民群众并肩作战的烽火岁月；也可以漫步芦苇荡间，乘一叶轻舟穿梭于碧波芦海，享受水乡的宁静野趣，领略独特的湿地风情；还可以来到横泾老街，漫步于上世纪三四十年代风貌的街巷，观看精彩的民俗表演，品尝地道的水乡美食，沉浸式体验淳朴悠然的江南民俗。沙家浜，是一幅自然与人文交织的画卷，更是一段值得用心感受的历史。愿您在这里度过一段充实而美好的时光！",
            'B': "各位朋友，大家好。在江南水乡沙家浜，曾镌刻下一段军民同心、共抗敌寇的红色记忆。这里有指导员郭建光的壮志凌云，有阿庆嫂的机智沉着，有沙奶奶的慈爱坚毅，也有与敌人周旋的惊心动魄。接下来，让我们循着京剧《沙家浜》的经典旋律，一同穿越烽火岁月，重温那段充满斗争智慧与深厚情谊的历史！",
            'C': "各位朋友，经典的唱腔余韵悠长，烽火里的故事依旧动人。我们刚刚一同重温了郭建光的壮志、沙奶奶的坚韧，也深深记住了阿庆嫂垒起七星灶的过人智慧，更读懂了那份跨越岁月的军民鱼水情。本场沙家浜京剧选段演出到此圆满结束，感谢您的驻足与陪伴，我们下次再会！",
            'D': "各位朋友，大家好！欢迎来到秋意浓浓的沙家浜！眼下芦苇泛黄、蟹肥菊香，正是赏秋好时候。接下来我们将登上手摇船畅游芦苇荡，登船时请务必注意脚下安全。祝愿大家在此度过一段难忘的秋日时光！"
        }
        
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
        self.state = "stopped"
        self.ramp_in_duration = 0.8        # 平滑进入时间
        self.move_to_initial_duration = 1.5  # 回到初始姿态的时间
        self.ramp_start_time = None
        self.start_time = None
        self.current_frame = 0

        # 当前反馈 (初始化为None，表示尚未获取到反馈)
        self.current_pose = None
        
        # 初始姿态（固定为零位）
        self.initial_pose = np.zeros(15, dtype=np.float32)
        print(f"? 初始姿态设置为零位: {self.initial_pose[:3]}")
        
        # 程序启动时的姿态
        self.startup_pose = None

        # 控制参数 - 优化以减少电机抖动
        self.base_kp_waist = 65.0   # 从100.0降低以减少抖动
        self.base_kp_arm = 40.0     # 从60.0降低以减少抖动

        self.base_kd_waist = 7    # 从5.0增加以提高阻尼
        self.base_kd_arm = 5      # 从3.0增加以提高阻尼
        
        # 动作幅度缩放因子 (减小动作幅度以提高平滑性和平衡性)
        self.action_scale_factor = 0.7  # 缩放到70%的动作幅度
        
        # 添加关节速度限制参数以提高平滑性
        self.max_joint_velocity = 1.0   # 最大关节速度 (rad/s)
        
        # 添加平滑插值参数
        self.smoothing_factor = 0.15  # 用于关节运动的平滑系数
        
        # 初始化标志
        self.has_sent_stop_cmd = False
        self.last_L1F1 = False  # 添加L1+F1按键状态跟踪
        self.last_F1Start = False  # 添加F1+Start按键状态跟踪
        self.last_F1Select = False  # 添加F1+Select按键状态跟踪
        self.last_F1L2 = False  # 添加F1+L2按键状态跟踪
        self.function_activated = False  # 功能激活状态
        self.in_main_loco_mode = True  # 机器人是否处于主运控模式
        self.voice_control_enabled = False  # 语音控制默认禁用
        self.tts_playing = False  # TTS播放状态标志

        # 音频播放控制事件
        self.audio_playback_stop_event = threading.Event()
        self.audio_playback_active = False  # 音频播放状态标志
        
        # 音频处理器
        try:
            self.audio_processor = G1AudioProcessor()
            print("? 音频处理器初始化成功")
        except Exception as e:
            print(f"? 音频处理器初始化失败: {e}")
            # 创建一个简化版本的音频处理器，确保基本功能可用
            class DummyAudioProcessor:
                def __init__(self):
                    try:
                        from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
                        self.audio_client = AudioClient()
                        self.audio_client.SetTimeout(5.0)
                        self.audio_client.Init()
                        print("? 音频客户端初始化成功")
                    except Exception as e:
                        print(f"? 音频客户端初始化失败: {e}")
                        self.audio_client = None
                
                def handle_wake_command(self):
                    pass
                    
                def handle_play_command(self, state):
                    return True
                    
                def handle_play_named_command(self, state, action_name, actions):
                    if actions:
                        return list(actions.keys())[0]
                    return None
                    
                def handle_stop_command(self, state):
                    return True
                    
                def handle_loop_command(self, loop):
                    return not loop
                    
                def process_audio_message(self, msg, handler):
                    pass
            
            self.audio_processor = DummyAudioProcessor()
        
        # 语音识别订阅器
        self.audio_subscriber = None

        # 初始化手臂动作客户端
        try:
            from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient, action_map
            self.arm_action_client = G1ArmActionClient()
            self.arm_action_client.SetTimeout(10.0)
            self.arm_action_client.Init()
            self.action_map = action_map
            print("? 手臂动作客户端初始化成功")
        except Exception as e:
            print(f"? 手臂动作客户端初始化失败: {e}")
            self.arm_action_client = None
            self.action_map = None

        # 外部进程句柄（用于避免重复启动）
        self._fastlio_proc = None
        self._fastlio_started_at = None

        self.load_actions()
        self.setup_publisher()
    
    def load_actions(self):
        """加载所有动作文件，支持加载分割的动作文件"""
        print(f"? 加载动作文件从目录: {self.action_dir}")
        
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
            if os.path.isdir(direction_path) and direction_dir in direction_map:
                direction_key = direction_map[direction_dir]  # 遥控器按键对应的方向键名称
                
                # 收集所有npz文件并按名称分组
                npz_files = glob.glob(os.path.join(direction_path, "*.npz"))
                action_groups = {}  # 用于存储动作组
                
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
                                print(f"??  动作文件 {npz_file} 缺少 'qpos' 字段，跳过")
                                continue
                                
                            qpos = data['qpos']
                            
                            # 支持 (T, 9) 自动扩展为 (T, 15)
                            if qpos.ndim == 2 and qpos.shape[1] == 9:
                                print(f"??  检测到 (T, 9) 格式，自动补 0 扩展为 (T, 15)")
                                zeros = np.zeros((qpos.shape[0], 6), dtype=qpos.dtype)
                                qpos = np.hstack([qpos, zeros])
                            elif qpos.ndim != 2 or qpos.shape[1] != 15:
                                print(f"??  动作文件 {npz_file} 格式不正确，跳过")
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
                        print(f"? 对动作序列进行预处理: {clean_name} (共{len(npz_file_group)}个文件)")
                        max_angle_delta = 0.25  # 可根据动作类型调整，如咏春拳等精细动作
                        merged_action_data = self._smooth_action_sequence(merged_action_data, max_angle_delta=max_angle_delta, min_fps=fps)
                        
                        # 查找对应的音频文件
                        audio_file = None
                        if clean_name in audio_map:
                            audio_file = audio_map[clean_name]
                        elif len(npz_file_group) > 0 and os.path.basename(npz_file_group[0]).replace(".npz", "") in audio_map:
                            base_name = os.path.basename(npz_file_group[0]).replace(".npz", "")
                            # 移除数字后缀查找音频文件
                            clean_base_name = ''.join([c for c in base_name if not c.isdigit()])
                            if clean_base_name in audio_map:
                                audio_file = audio_map[clean_base_name]
                        
                        # 保存动作
                        self.actions[direction_key] = {  # 使用遥控器按键方向键名称作为键
                            'file': npz_file_group[0],  # 使用第一个文件作为代表
                            'data': merged_action_data,
                            'fps': fps,
                            'dt': 1.0 / fps,
                            'name': clean_name,
                            'source_dir': direction_dir,  # 保存源目录名，用于调试
                            'audio_file': audio_file,  # 保存对应的音频文件路径
                            'original_files': npz_file_group  # 保存原始文件列表
                        }
                        
                        print(f"? 加载动作: {direction_key} (来自 {direction_dir}/) -> '{clean_name}' | 帧数: {len(merged_action_data)} | fps: {fps:.1f} | 文件数: {len(npz_file_group)}")
                        if audio_file:
                            print(f"? 关联音频文件: {audio_file}")
                    except Exception as e:
                        print(f"? 加载动作文件组 {clean_name} 失败: {e}")
        
        if not self.actions:
            print("??  未找到任何动作文件")
        else:
            print(f"? 成功加载 {len(self.actions)} 个动作")
            # 显示加载的动作详情
            for direction_key, action in self.actions.items():
                print(f"  ? {direction_key}: {action['name']} (来自 {action['source_dir']}/)")
                if action.get('audio_file'):
                    print(f"     ? 音频: {action['audio_file']}")
                if 'original_files' in action and len(action['original_files']) > 1:
                    print(f"     ? 分割文件: {len(action['original_files'])} 个")
    
    def _get_file_number(self, filepath):
        """
        从文件路径中提取数字后缀
        
        Args:
            filepath: 文件路径
            
        Returns:
            int: 数字后缀，如果没有则返回0
        """
        filename = os.path.basename(filepath).replace(".npz", "")
        # 提取文件名中的数字
        numbers = ''.join([c for c in filename if c.isdigit()])
        return int(numbers) if numbers else 0
    
    def _smooth_action_sequence(self, action_data, max_angle_delta=0.25, min_fps=30.0):
        """
        对动作序列进行平滑处理，在关节角度变化剧烈的地方插入中间帧
        
        Args:
            action_data: 原始动作数据 (T, 15)
            max_angle_delta: 最大允许的关节角度变化 (弧度)
            min_fps: 最小帧率
            
        Returns:
            np.ndarray: 平滑后的动作数据
        """
        if len(action_data) < 2:
            return action_data
            
        smoothed_frames = [action_data[0]]  # 从第一帧开始
        
        for i in range(1, len(action_data)):
            prev_frame = action_data[i-1]
            curr_frame = action_data[i]
            
            # 计算关节角度变化
            angle_deltas = np.abs(curr_frame - prev_frame)
            max_delta = np.max(angle_deltas)
            
            # 如果角度变化过大，插入中间帧
            if max_delta > max_angle_delta:
                # 计算需要插入的帧数
                num_interpolated = int(np.ceil(max_delta / max_angle_delta))
                num_interpolated = min(num_interpolated, 10)  # 限制最大插入帧数
                
                # 线性插值
                for j in range(1, num_interpolated + 1):
                    ratio = j / (num_interpolated + 1)
                    interpolated_frame = prev_frame + ratio * (curr_frame - prev_frame)
                    smoothed_frames.append(interpolated_frame)
            
            smoothed_frames.append(curr_frame)
        
        return np.array(smoothed_frames, dtype=np.float32)
    
    def play_action(self, direction, speed=1.0):
        """
        播放指定方向的动作
        
        Args:
            direction: 动作方向 ('Up', 'Down', 'Left', 'Right', 'A', 'B', 'X', 'Y')
            speed: 播放速度倍数
        """
        if direction not in self.actions:
            print(f"? 未找到方向 '{direction}' 的动作")
            return
            
        if self.state != "stopped":
            print(f"??  当前正在播放动作，无法播放新动作")
            return
            
        action = self.actions[direction]
        print(f"? 开始播放动作: {action['name']} (方向: {direction})")
        
        # 设置动作参数
        self.current_action = action
        self.fps = action['fps'] * speed
        self.dt = 1.0 / self.fps
        self.current_frame = 0
        self.state = "playing"
        self.start_time = time.time()
        self.ramp_start_time = None
        
        # 播放关联的音频
        self._play_associated_audio(action)
    
    def _play_associated_audio(self, action):
        """播放动作关联的音频文件"""
        if not action.get('audio_file'):
            return
            
        audio_file = action['audio_file']
        if not os.path.exists(audio_file):
            print(f"??  音频文件不存在: {audio_file}")
            return
            
        print(f"? 播放关联音频: {audio_file}")
        # 这里可以添加音频播放逻辑
    
    def stop_play(self, no_tts=False):
        """停止播放并回到初始姿态"""
        if self.state == "stopped":
            return
            
        print("? 停止动作播放")
        self.state = "stopping"
        self.current_action = None
        self.start_time = None
        self.ramp_start_time = time.time()
        
        # 停止音频播放
        self._stop_audio_playback()
    
    def _stop_audio_playback(self):
        """停止音频播放"""
        self.audio_playback_stop_event.set()
        self.audio_playback_active = False
    
    def setup_publisher(self):
        """设置发布器"""
        try:
            self.publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
            self.publisher.Init()
            self.low_cmd = LowCmd_()
            print("? 低层命令发布器初始化成功")
        except Exception as e:
            print(f"? 低层命令发布器初始化失败: {e}")
    
    def _send_pose(self, q, dq=None, kp_scale=1.0, kd_scale=1.0):
        """发送姿态控制命令"""
        if self.publisher is None or self.low_cmd is None:
            return
            
        # 在stopped状态下不发送任何控制指令
        if self.state == "stopped":
            return
            
        # 设置关节位置
        for i in range(min(len(q), 15)):
            self.low_cmd.motor_cmd[i].q = q[i]
            self.low_cmd.motor_cmd[i].dq = dq[i] if dq is not None else 0.0
            
            # 根据关节类型设置不同的控制参数
            if i == G1JointIndex.WaistYaw:
                self.low_cmd.motor_cmd[i].kp = self.base_kp_waist * kp_scale
                self.low_cmd.motor_cmd[i].kd = self.base_kd_waist * kd_scale
            elif i >= G1JointIndex.LeftShoulderPitch:
                self.low_cmd.motor_cmd[i].kp = self.base_kp_arm * kp_scale
                self.low_cmd.motor_cmd[i].kd = self.base_kd_arm * kd_scale
            else:
                self.low_cmd.motor_cmd[i].kp = 0.0
                self.low_cmd.motor_cmd[i].kd = 0.0
                
            self.low_cmd.motor_cmd[i].tau = 0.0
        
        # 设置CRC校验
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        
        # 发布命令
        self.publisher.Write(self.low_cmd)
    
    def update(self):
        """更新动作播放状态"""
        if self.state == "stopped":
            return
            
        current_time = time.time()
        
        if self.state == "playing" and self.current_action:
            # 计算当前帧
            elapsed_time = current_time - self.start_time
            target_frame = int(elapsed_time * self.fps)
            
            if target_frame >= len(self.current_action['data']):
                # 动作播放完成
                self.stop_play()
                return
                
            # 发送当前帧的姿态
            current_pose = self.current_action['data'][target_frame]
            self._send_pose(current_pose)
            
        elif self.state == "stopping":
            # 平滑回到初始姿态
            if self.ramp_start_time is None:
                self.ramp_start_time = current_time
                
            elapsed = current_time - self.ramp_start_time
            if elapsed >= self.move_to_initial_duration:
                # 回到初始姿态完成
                self.state = "stopped"
                self._send_pose(self.initial_pose)
                print("? 已回到初始姿态")
            else:
                # 插值回到初始姿态
                ratio = elapsed / self.move_to_initial_duration
                if self.current_pose is not None:
                    interpolated_pose = self.current_pose + ratio * (self.initial_pose - self.current_pose)
                    self._send_pose(interpolated_pose)
    
    def _start_fastlio_navigation(self):
        """启动fastlio导航"""
        try:
            # 检查是否已经启动过
            if self._fastlio_proc is not None and self._fastlio_proc.poll() is None:
                print("??  fastlio导航已经在运行中")
                return
                
            # 启动fastlio导航
            cmd = [
                "bash", "-lc",
                "cd /home/unitree/HongTu/G1Nav2D && source /opt/ros/noetic/setup.bash && "
                "roslaunch fastlio2 gridmap_load.launch use_rviz:=false"
            ]
            
            self._fastlio_proc = subprocess.Popen(cmd)
            self._fastlio_started_at = time.time()
            print("? fastlio导航启动成功")
            
        except Exception as e:
            print(f"? 启动fastlio导航失败: {e}")
    
    def _can_trigger_after_nav(self, wait_seconds: float = 10.0) -> bool:
        """检查是否可以在导航启动后触发动作"""
        if self._fastlio_started_at is None:
            return False
        return time.time() - self._fastlio_started_at >= wait_seconds
