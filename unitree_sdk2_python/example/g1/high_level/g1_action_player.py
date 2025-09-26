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
import shutil
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
        
        # 保存原始音量设置
        self.original_volume = None
        self._save_original_volume()
        
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
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        if direction not in self.actions:
            print(f"? 未找到方向 '{direction}' 的动作")
            return False
            
        if self.state != "stopped":
            print(f"??  当前正在播放动作，无法播放新动作")
            return False
            
        action = self.actions[direction]
        print(f"? 开始播放动作: {action['name']} (方向: {direction})")
        
        # 播放开始提示音
        tts_start_time = None
        try:
            # 添加防重复机制
            if not hasattr(self, '_last_play_tts_time'):
                self._last_play_tts_time = 0
            current_time = time.time()
            if current_time - self._last_play_tts_time > 3.0:  # 至少间隔3秒
                if self.audio_processor and hasattr(self.audio_processor, 'audio_client'):
                    self.audio_processor.audio_client.TtsMaker(f"我给大家表演{action['name']}", 0)
                    tts_start_time = current_time
                self._last_play_tts_time = current_time
        except Exception as e:
            print(f"? 播放开始提示失败: {e}")
        
        # 等待TTS播报完成（估计时间约5秒）
        if tts_start_time is not None:
            tts_wait_time = 5.0  # 等待5秒确保TTS播报完成
            time_to_wait = tts_wait_time - (time.time() - tts_start_time)
            if time_to_wait > 0:
                time.sleep(time_to_wait)
            print("? TTS播报已完成")
        
        # 添加额外延迟确保TTS完全结束
        time.sleep(0.5)
        
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
        
        return True
    
    def init_to_zero_position(self):
        """
        程序启动后主动执行平滑移动到预设安全位置
        """
        print("? 程序启动，开始执行初始化到预设安全位置...")
        
        # 播放初始化提示音
        try:
            # 添加防重复机制
            if not hasattr(self, '_last_init_tts_time'):
                self._last_init_tts_time = 0
            current_time = time.time()
            if current_time - self._last_init_tts_time > 3.0:  # 至少间隔3秒
                if self.audio_processor and hasattr(self.audio_processor, 'audio_client'):
                    self.audio_processor.audio_client.TtsMaker("系统启动，正在初始化", 0)
                self._last_init_tts_time = current_time
        except Exception as e:
            print(f"? 音频初始化提示失败: {e}")
        
        # 等待获取实际的当前位置反馈
        wait_start = time.time()
        while self.current_pose is None and (time.time() - wait_start) < 5.0:  # 等待最多5秒
            print("? 等待接收实际关节位置反馈...")
            time.sleep(0.1)
            
        if self.current_pose is None:
            # 如果超时仍未收到反馈，不能继续执行初始化
            print("? 超时未收到反馈，无法执行初始化")
            try:
                if self.audio_processor and hasattr(self.audio_processor, 'audio_client'):
                    self.audio_processor.audio_client.TtsMaker("初始化失败", 0)
            except Exception as e:
                print(f"? 错误提示播放失败: {e}")
            return
            
        print(f"? 收到实际关节位置反馈: {self.current_pose[:3]}")
        
        # 定义预设的安全初始姿态
        target_pose = np.zeros(15, dtype=np.float32)
        # 腰部保持零位
        # 左臂预设位置
        target_pose[1:8] = [0.292, 0.220, -0.011, 0.984, 0.097, 0.022, -0.036]
        # 右臂预设位置
        target_pose[8:15] = [0.290, -0.219, 0.025, 0.974, -0.088, 0.032, 0.021]
        
        start_pose = self.current_pose.copy()
        
        print(f"? 起始位置: {start_pose[:3]}")
        print(f"? 目标位置: {target_pose[:3]}")
        
        # 检查是否已经在目标位置附近，如果是则跳过平滑过渡
        position_diff = np.linalg.norm(start_pose - target_pose)
        print(f"? 起始位置与目标位置差异: {position_diff:.3f}")
        if position_diff < 0.05:  # 如果差异小于0.05弧度，则认为已经在位置上
            print("? 已经在目标位置附近，跳过平滑过渡")
            self.startup_pose = target_pose.copy()
            # 发送一次位置命令确保位置稳定
            self._send_pose(target_pose, dq=np.zeros(15), kp_scale=0.3, kd_scale=1.0)
            time.sleep(0.1)
            # 初始化完成后设置状态为stopped，避免持续发送指令
            self.state = "stopped"
            # 主动释放手臂
            if self.arm_action_client and self.action_map:
                try:
                    self.arm_action_client.ExecuteAction(self.action_map.get("release arm"))
                    print("? 手臂已释放")
                except Exception as e:
                    print(f"? 释放手臂时出错: {e}")
            return
            
        # 平滑过渡到目标位置
        print("? 开始平滑过渡到预设安全位置...")
        duration = 3.0  # 减少过渡时间从4秒到3秒，减少等待时间
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
            time.sleep(0.02)  # 50Hz控制频率，保持不变
            
        # 确保最终位置
        self._send_pose(target_pose, dq=np.zeros(15), kp_scale=0.3, kd_scale=1.0)
        time.sleep(0.1)
        
        # 保存这个预设位置作为初始姿态
        self.startup_pose = target_pose.copy()
        print("? 初始化到预设安全位置完成")
        print(f"? 当前位置: {target_pose[:3]}")
        
        # 初始化完成后设置状态为stopped，避免持续发送指令
        self.state = "stopped"
        
        # 主动释放手臂
        if self.arm_action_client and self.action_map:
            try:
                self.arm_action_client.ExecuteAction(self.action_map.get("release arm"))
                print("? 手臂已释放")
            except Exception as e:
                print(f"? 释放手臂时出错: {e}")
        
        # 初始化完成提示
        try:
            if self.audio_processor and hasattr(self.audio_processor, 'audio_client'):
                self.audio_processor.audio_client.TtsMaker("初始化完成，系统就绪", 0)
                time.sleep(1)
        except Exception as e:
            print(f"? 初始化完成提示失败: {e}")
    
    def _play_associated_audio(self, action):
        """
        播放与动作关联的音频文件
        
        Args:
            action: 动作数据字典
        """
        try:
            if not action.get('audio_file'):
                return
                
            audio_file = action['audio_file']
            if not os.path.exists(audio_file):
                print(f"??  音频文件不存在: {audio_file}")
                return
                
            print(f"🎵 播放关联音频: {audio_file}")
            # 停止可能正在播放的任何音频
            if self.audio_processor and hasattr(self.audio_processor, 'audio_client'):
                self.audio_processor.audio_client.PlayStop("g1_client")
            
            # 重置音频播放控制事件
            self.audio_playback_stop_event.clear()
            self.audio_playback_active = True  # 在开始播放前设置为活跃状态
            
            # 导入音频处理函数
            try:
                from wav import read_wav, get_wav_duration
                
                # 获取音频时长
                audio_duration = get_wav_duration(audio_file)
                if audio_duration > 0:
                    self.audio_duration = audio_duration
                    print(f"[DEBUG] 音频时长: {audio_duration:.2f} 秒 ({int(audio_duration//60)}分{int(audio_duration%60)}秒)")
                
                # 读取音频文件
                pcm_list, sample_rate, num_channels, is_ok = read_wav(audio_file)
                print(f"[DEBUG] Read success: {is_ok}")
                print(f"[DEBUG] Sample rate: {sample_rate} Hz")
                print(f"[DEBUG] Channels: {num_channels}")
                print(f"[DEBUG] PCM byte length: {len(pcm_list)}")
                
                # 检查是否需要转换音频格式
                if not is_ok:
                    print("[ERROR] Failed to read WAV file")
                    self.audio_playback_active = False
                else:
                    # 仅在确实需要转换时才进行转换（采样率不是16000Hz或声道数不是1）
                    if sample_rate != 16000 or num_channels != 1:
                        print(f"[INFO] 音频格式需要转换: {sample_rate}Hz, {num_channels}声道 -> 16000Hz, 单声道")
                        converted_pcm_list = self._convert_audio_format(pcm_list, sample_rate, num_channels)
                        if converted_pcm_list is not None:
                            pcm_list = converted_pcm_list
                            print(f"[INFO] 音频转换完成: {len(pcm_list)} 个样本")
                        else:
                            print("[ERROR] 音频格式转换失败")
                            self.audio_playback_active = False
                            return False
                    else:
                        print("[INFO] 音频格式已符合要求 (16000Hz, 单声道)，无需转换")
                    
                    # 设置音频播放状态为活跃
                    self.audio_playback_active = True
                    
                    # 在单独的线程中播放音频，避免阻塞动作播放
                    import threading
                    def play_audio():
                        # 使用官方的play_pcm_stream函数
                        if self.audio_processor and hasattr(self.audio_processor, 'audio_client'):
                            self._play_pcm_stream_interruptible(
                                self.audio_processor.audio_client, 
                                pcm_list, 
                                "g1_client", 
                                chunk_size=24000,   # 使用官方推荐的块大小
                                sleep_time=0.75,    # 使用官方推荐的休眠时间
                                verbose=False
                            )
                        # 播放完成后更新状态
                        self.audio_playback_active = False
                    
                    audio_thread = threading.Thread(target=play_audio)
                    audio_thread.daemon = True
                    audio_thread.start()
                    print("✅ 音频播放已启动")
                    # 添加一个小延迟，确保音频线程启动
                    time.sleep(0.1)
            except ImportError:
                print("⚠️ wav模块未找到，跳过音频播放")
                self.audio_playback_active = False
        except Exception as e:
            print(f"❌ 播放音频文件失败: {e}")
            self.audio_playback_active = False
    
    def stop_play(self, no_tts=False):
        """停止播放并回到初始姿态"""
        if self.state == "stopped":
            return
            
        # 只有在播放状态才进入回到初始姿态的流程
        if self.state in ["playing", "ramp_in"]:
            if not no_tts:
                print(f"? 动作结束，进入平滑回到初始姿态流程")
                # 立即播放结束提示音，不等待，添加防重复机制
                try:
                    # 添加防重复机制
                    if not hasattr(self, '_last_stop_tts_time'):
                        self._last_stop_tts_time = 0
                    current_time = time.time()
                    if current_time - self._last_stop_tts_time > 3.0:  # 至少间隔3秒
                        if self.current_action:
                            # 安全访问音频客户端
                            if self.audio_processor and hasattr(self.audio_processor, 'audio_client'):
                                self.audio_processor.audio_client.TtsMaker(f"动作{self.current_action['name']}播放结束，正在回到初始位置", 0)
                        else:
                            # 安全访问音频客户端
                            if self.audio_processor and hasattr(self.audio_processor, 'audio_client'):
                                self.audio_processor.audio_client.TtsMaker("动作播放结束，正在回到初始位置", 0)
                        self._last_stop_tts_time = current_time
                except Exception as e:
                    print(f"? 播放结束提示失败: {e}")
            else:
                print(f"? 动作结束，进入平滑回到初始姿态流程（无TTS提示）")
            
            # 停止音频播放
            self._stop_audio_playback()
            
            self.state = "move_to_initial"
            self.ramp_start_time = time.time()
        else:
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
    
    def _convert_audio_format(self, pcm_list, sample_rate, num_channels):
        """
        转换音频格式为16kHz单声道，使用高质量重采样算法
        
        Args:
            pcm_list: 原始PCM数据列表
            sample_rate: 原始采样率
            num_channels: 原始声道数
            
        Returns:
            转换后的PCM数据列表，如果失败则返回None
        """
        try:
            from wav import convert_wav
            return convert_wav(pcm_list, sample_rate, num_channels)
        except Exception as e:
            print(f"[ERROR] 音频格式转换失败: {e}")
            return None
    
    def _play_pcm_stream_interruptible(self, client, pcm_list, stream_name="example", chunk_size=24000, sleep_time=0.75, verbose=False):
        """
        支持中断的PCM音频流播放功能
        
        Parameters:
            client: 音频客户端
            pcm_list: PCM音频数据列表
            stream_name: 流名称
            chunk_size: 每个块的大小
            sleep_time: 块之间的休眠时间
            verbose: 是否显示详细信息
        """
        try:
            import time
            import struct
            import array
            
            # 将int16列表转换为字节数据
            if isinstance(pcm_list, list):
                pcm_array = array.array('h', pcm_list)  # 'h' 表示有符号短整型 (int16)
                pcm_data = pcm_array.tobytes()
            else:
                pcm_data = pcm_list
            
            total_bytes = len(pcm_data)
            bytes_sent = 0
            
            if verbose:
                print(f"[AUDIO] 开始播放PCM流: {stream_name}, 总字节数: {total_bytes}")
            
            # 分块播放音频数据
            while bytes_sent < total_bytes and not self.audio_playback_stop_event.is_set():
                # 计算当前块的大小
                remaining_bytes = total_bytes - bytes_sent
                current_chunk_size = min(chunk_size, remaining_bytes)
                
                # 获取当前块的数据
                chunk_data = pcm_data[bytes_sent:bytes_sent + current_chunk_size]
                
                # 发送音频块
                try:
                    result = client.PlayPcmStream(stream_name, chunk_data)
                    if result != 0:
                        if verbose:
                            print(f"[AUDIO] 播放块失败，错误码: {result}")
                        break
                except Exception as e:
                    if verbose:
                        print(f"[AUDIO] 播放块异常: {e}")
                    break
                
                bytes_sent += current_chunk_size
                
                if verbose:
                    progress = (bytes_sent / total_bytes) * 100
                    print(f"[AUDIO] 播放进度: {progress:.1f}% ({bytes_sent}/{total_bytes})")
                
                # 检查是否需要停止
                if self.audio_playback_stop_event.is_set():
                    if verbose:
                        print("[AUDIO] 收到停止信号，中断播放")
                    break
                
                # 休眠一段时间
                time.sleep(sleep_time)
            
            # 播放完成或中断
            if bytes_sent >= total_bytes:
                if verbose:
                    print("[AUDIO] PCM流播放完成")
            else:
                if verbose:
                    print(f"[AUDIO] PCM流播放中断，已播放: {bytes_sent}/{total_bytes}")
                    
        except Exception as e:
            print(f"[AUDIO] PCM流播放异常: {e}")
    
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
            
        # 限制发送频率以减少CPU使用
        current_time = time.time()
        if not hasattr(self, '_last_send_time'):
            self._last_send_time = 0
            
        # 在动作播放期间提高发送频率到25ms一次 (~40Hz)，其他时候保持40ms
        # 降低频率以减少抖动
        if self.state in ["ramp_in", "playing", "move_to_initial"]:
            send_interval = 0.025  # 动作播放期间和回到初始位置时都使用40Hz
        else:
            send_interval = 0.04   # 其他时候25Hz
            
        if current_time - self._last_send_time < send_interval:
            return
            
        self._last_send_time = current_time
            
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
    
    def update_low_frequency(self):
        """
        低频更新函数，用于在功能未激活时减少CPU占用
        """
        # 限制update_low_frequency函数的执行频率
        current_time = time.time()
        
        # 使用实例属性来存储上次调用时间
        if not hasattr(self, '_last_low_freq_update_call'):
            self._last_low_freq_update_call = 0
            
        # 限制调用频率为200ms一次，降低CPU占用
        if current_time - self._last_low_freq_update_call < 0.2:
            return
        self._last_low_freq_update_call = current_time
        
        # 如果尚未获取到当前位置反馈，使用零位作为默认位置
        if self.current_pose is None:
            self.current_pose = np.zeros(15, dtype=np.float32)
            return

        # 在停止状态下不发送任何控制指令，让遥控器正常工作
        if self.state == "stopped":
            return

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
                print(f"? 动作播放完毕（{len(self.current_action['data'])} 帧），进入退出流程")
                
                # 立即播放完成提示音，添加防重复机制
                try:
                    # 添加防重复机制
                    if not hasattr(self, '_last_finish_tts_time'):
                        self._last_finish_tts_time = 0
                    current_time = time.time()
                    if current_time - self._last_finish_tts_time > 3.0:  # 至少间隔3秒
                        if self.current_action:
                            if self.audio_processor and hasattr(self.audio_processor, 'audio_client'):
                                self.audio_processor.audio_client.TtsMaker(f"动作{self.current_action['name']}播放完毕，正在回到初始位置", 0)
                        else:
                            if self.audio_processor and hasattr(self.audio_processor, 'audio_client'):
                                self.audio_processor.audio_client.TtsMaker("动作播放完毕，正在回到初始位置", 0)
                        self._last_finish_tts_time = current_time
                except Exception as e:
                    print(f"? 播放完成提示失败: {e}")
                
                self.stop_play()
                return
                
            # 发送当前帧的姿态
            current_pose = self.current_action['data'][target_frame]
            self._send_pose(current_pose)
            
        elif self.state == "move_to_initial":
            # 平滑回到初始姿态
            if self.ramp_start_time is None:
                self.ramp_start_time = current_time
                
            elapsed = current_time - self.ramp_start_time
            if elapsed >= self.move_to_initial_duration:
                # 回到初始姿态完成
                self.state = "stopped"
                self._send_pose(self.initial_pose)
                print("? 已回到初始姿态")
                
                # 播放完成提示音
                try:
                    # 添加防重复机制
                    if not hasattr(self, '_last_complete_tts_time'):
                        self._last_complete_tts_time = 0
                    current_time = time.time()
                    if current_time - self._last_complete_tts_time > 3.0:  # 至少间隔3秒
                        if self.current_action:
                            if self.audio_processor and hasattr(self.audio_processor, 'audio_client'):
                                self.audio_processor.audio_client.TtsMaker(f"已回到初始位置，{self.current_action['name']}动作完成", 0)
                        else:
                            if self.audio_processor and hasattr(self.audio_processor, 'audio_client'):
                                self.audio_processor.audio_client.TtsMaker("已回到初始位置", 0)
                        self._last_complete_tts_time = current_time
                except Exception as e:
                    print(f"? 播放完成提示失败: {e}")
            else:
                # 插值回到初始姿态
                ratio = elapsed / self.move_to_initial_duration
                if self.current_pose is not None:
                    interpolated_pose = self.current_pose + ratio * (self.initial_pose - self.current_pose)
                    self._send_pose(interpolated_pose)
                    
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
        """
        启动 fastlio 导航: roslaunch fastlio navigation use_rviz:=false
        - 优先在可用终端中打开并打印输出
        - 若无图形终端可用，则在后台运行并输出到日志文件
        - 避免重复启动
        """
        try:
            # 若已有运行中的进程，避免重复启动
            if self._fastlio_proc is not None and self._fastlio_proc.poll() is None:
                print("[fastlio] 已在运行，跳过重复启动")
                return

            # 准备日志目录与文件
            log_dir = "/home/unitree/HongTu/PythonProject/point_nav/logs"
            os.makedirs(log_dir, exist_ok=True)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            log_file = os.path.join(log_dir, f"fastlio_{ts}.log")

            launch_cmd = "roslaunch fastlio navigation.launch use_rviz:=false"

            # 根据可用终端选择启动方式
            term = shutil.which("gnome-terminal")
            xterm = shutil.which("xterm")
            display_ok = bool(os.environ.get("DISPLAY"))

            if display_ok and term:
                # 在 gnome-terminal 中启动，并将输出 tee 到日志，同时保留终端
                cmd = [
                    term,
                    "--",
                    "bash",
                    "-lc",
                    f"{launch_cmd} 2>&1 | tee -a '{log_file}'; exec bash"
                ]
                print(f"[fastlio] 使用 gnome-terminal 启动，日志: {log_file}")
                self._fastlio_proc = subprocess.Popen(cmd)
                self._fastlio_started_at = time.time()
            elif display_ok and xterm:
                # 在 xterm 中启动，-hold 保持窗口
                cmd = [
                    xterm,
                    "-hold",
                    "-e",
                    "bash",
                    "-lc",
                    f"{launch_cmd} 2>&1 | tee -a '{log_file}'"
                ]
                print(f"[fastlio] 使用 xterm 启动，日志: {log_file}")
                self._fastlio_proc = subprocess.Popen(cmd)
                self._fastlio_started_at = time.time()
            else:
                # 后台运行，输出到日志
                print(f"[fastlio] 无可用终端，后台运行。日志: {log_file}")
                log_fh = open(log_file, "a", buffering=1)
                self._fastlio_proc = subprocess.Popen(
                    ["bash", "-lc", launch_cmd],
                    stdout=log_fh,
                    stderr=subprocess.STDOUT,
                    preexec_fn=os.setsid
                )
                self._fastlio_started_at = time.time()
                print(f"[fastlio] 后台启动成功，PID: {self._fastlio_proc.pid}")

        except Exception as e:
            print(f"[fastlio] 启动失败: {e}")
    
    def _can_trigger_after_nav(self, wait_seconds: float = 10.0) -> bool:
        """检查是否可以在导航启动后触发动作"""
        if self._fastlio_started_at is None:
            return False
        return time.time() - self._fastlio_started_at >= wait_seconds
    
    def _shutdown_navigation_systems(self):
        """关闭所有导航建图相关程序"""
        try:
            print("🛑 开始关闭所有导航建图相关程序...")
            
            # 关闭fastlio进程
            if self._fastlio_proc is not None:
                print("🛑 关闭fastlio导航进程...")
                self._fastlio_proc.terminate()
                try:
                    self._fastlio_proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    print("⚠️ fastlio进程未正常退出，强制终止...")
                    self._fastlio_proc.kill()
                self._fastlio_proc = None
                self._fastlio_started_at = None
            
            # 关闭ROS相关进程
            print("🛑 关闭ROS导航相关进程...")
            try:
                # 关闭move_base
                subprocess.run(["pkill", "-f", "move_base"], check=False)
                # 关闭amcl
                subprocess.run(["pkill", "-f", "amcl"], check=False)
                # 关闭map_server
                subprocess.run(["pkill", "-f", "map_server"], check=False)
                # 关闭fastlio相关进程
                subprocess.run(["pkill", "-f", "fastlio"], check=False)
                # 关闭重定位进程
                subprocess.run(["pkill", "-f", "auto_relocalize"], check=False)
                # 关闭rviz
                subprocess.run(["pkill", "-f", "rviz"], check=False)
                print("✅ 导航相关进程已关闭")
            except Exception as e:
                print(f"⚠️ 关闭ROS进程时出错: {e}")
            
            print("✅ 所有导航建图相关程序已关闭")
            
        except Exception as e:
            print(f"❌ 关闭导航系统失败: {e}")
    
    def _start_fastlio_navigation_with_monitoring(self):
        """启动fastlio导航并监测重定位状态"""
        try:
            # 先关闭可能存在的旧进程
            self._shutdown_navigation_systems()
            time.sleep(2)  # 等待进程完全关闭
            
            print("🚀 启动fastlio导航并开始重定位监测...")
            
            # 启动fastlio导航
            cmd = [
                "bash", "-lc",
                "cd /home/unitree/HongTu/G1Nav2D && source /opt/ros/noetic/setup.bash && "
                "roslaunch fastlio2 gridmap_load.launch use_rviz:=false"
            ]
            
            self._fastlio_proc = subprocess.Popen(cmd)
            self._fastlio_started_at = time.time()
            print("✅ fastlio导航启动成功")
            
            # 在单独线程中监测重定位状态
            import threading
            monitoring_thread = threading.Thread(target=self._monitor_relocalization)
            monitoring_thread.daemon = True
            monitoring_thread.start()
            
        except Exception as e:
            print(f"❌ 启动fastlio导航失败: {e}")
    
    def _monitor_relocalization(self):
        """监测重定位状态"""
        try:
            print("🔍 开始监测重定位状态...")
            
            # 等待5秒让系统启动
            time.sleep(5)
            
            # 监测20秒
            start_time = time.time()
            timeout = 20.0
            
            while time.time() - start_time < timeout:
                if self._check_relocalization_status():
                    print("✅ 重定位成功!")
                    if hasattr(self, 'audio_processor') and hasattr(self.audio_processor, 'audio_client'):
                        self.audio_processor.audio_client.TtsMaker("重定位成功，导航已就绪，十秒后可按方向键触发动作", 0)
                    return True
                
                time.sleep(1)  # 每秒检查一次
            
            # 20秒后仍未成功
            print("❌ 重定位失败，关闭导航程序")
            self._shutdown_navigation_systems()
            if hasattr(self, 'audio_processor') and hasattr(self.audio_processor, 'audio_client'):
                self.audio_processor.audio_client.TtsMaker("重定位失败，导航程序已关闭，请把机器人移动到指定位置重新启动程序", 0)
            
            return False
            
        except Exception as e:
            print(f"❌ 重定位监测失败: {e}")
            return False
    
    def _check_relocalization_status(self):
        """检查重定位状态"""
        try:
            # 检查是否有重定位成功的标志
            # 这里可以通过检查ROS话题或服务来判断重定位状态
            import subprocess
            
            # 检查amcl是否正在运行且定位成功
            result = subprocess.run(
                ["rostopic", "echo", "/amcl_pose", "-n", "1", "--timeout", "2"],
                capture_output=True, text=True, check=False
            )
            
            if result.returncode == 0 and result.stdout.strip():
                # 检查定位质量
                result = subprocess.run(
                    ["rostopic", "echo", "/amcl_pose", "-n", "1", "--timeout", "2"],
                    capture_output=True, text=True, check=False
                )
                
                if result.returncode == 0:
                    # 简单检查：如果能够获取到amcl_pose，认为定位成功
                    print("📍 检测到定位成功")
                    return True
            
            # 也可以检查其他指标，比如tf变换
            result = subprocess.run(
                ["rostopic", "echo", "/tf", "-n", "1", "--timeout", "2"],
                capture_output=True, text=True, check=False
            )
            
            if result.returncode == 0 and "map" in result.stdout and "base_link" in result.stdout:
                print("📍 检测到tf变换正常，定位成功")
                return True
            
            return False
            
        except Exception as e:
            print(f"⚠️ 检查重定位状态时出错: {e}")
            return False
    
    def _save_original_volume(self):
        """保存原始音量设置"""
        try:
            if hasattr(self, 'audio_processor') and hasattr(self.audio_processor, 'audio_client'):
                code, volume_data = self.audio_processor.audio_client.GetVolume()
                if code == 0 and volume_data:
                    self.original_volume = volume_data.get('volume', 100)
                    print(f"🔊 已保存原始音量: {self.original_volume}")
                else:
                    self.original_volume = 100  # 默认音量
                    print(f"⚠️ 无法获取当前音量，使用默认值: {self.original_volume}")
            else:
                self.original_volume = 100
                print(f"⚠️ 音频客户端不可用，使用默认音量: {self.original_volume}")
        except Exception as e:
            self.original_volume = 100
            print(f"⚠️ 保存原始音量失败: {e}，使用默认音量: {self.original_volume}")
    
    def _set_volume(self, volume):
        """设置音量"""
        try:
            if hasattr(self, 'audio_processor') and hasattr(self.audio_processor, 'audio_client'):
                code = self.audio_processor.audio_client.SetVolume(volume)
                if code == 0:
                    print(f"🔊 音量已设置为: {volume}")
                    return True
                else:
                    print(f"⚠️ 设置音量失败，错误码: {code}")
                    return False
            else:
                print("⚠️ 音频客户端不可用，无法设置音量")
                return False
        except Exception as e:
            print(f"⚠️ 设置音量时出错: {e}")
            return False
    
    def _set_volume_to_minimum(self):
        """将音量设置为最小"""
        return self._set_volume(0)
    
    def _restore_original_volume(self):
        """恢复原始音量"""
        if self.original_volume is not None:
            return self._set_volume(self.original_volume)
        else:
            return self._set_volume(100)  # 默认音量
    
    def handle_audio_command(self, command_type, text, action_name=None):
        """
        处理音频命令
        
        Args:
            command_type: 命令类型 ('wake', 'play', 'play_named', 'stop', 'loop')
            text: 处理后的文本
            action_name: 动作名称（仅在play_named命令中使用）
        """
        if command_type == 'wake':
            self.audio_processor.handle_wake_command()
        elif command_type == 'play':
            if self.audio_processor.handle_play_command(self.state):
                # 随机播放一个动作作为示例
                if self.actions:
                    direction = list(self.actions.keys())[0]  # 播放第一个动作
                    self.play_action(direction)
        elif command_type == 'play_named':
            # 使用传入的action_name参数
            if action_name:
                # 处理播放指定名称的动作
                action_key = self.audio_processor.handle_play_named_command(
                    self.state, action_name, self.actions)
                if action_key:
                    self.play_action(action_key)
        elif command_type == 'stop':
            if self.audio_processor.handle_stop_command(self.state):
                self.stop_play()
        elif command_type == 'loop':
            self.loop = self.audio_processor.handle_loop_command(self.loop)

    def _play_tts_with_wait(self, text, speaker_id=0):
        """
        播放TTS文本并等待播放完成
        
        Args:
            text: 要播放的文本
            speaker_id: 说话人ID
        """
        try:
            # 根据测试，单次TTS文本建议不超过150字符以确保稳定性
            max_length = 150
            if len(text) > max_length:
                # 更智能的文本分割方法
                segments = []
                # 先按句子分割
                import re
                sentences = re.split(r'[。！？；;.!?;]', text)
                
                current_segment = ""
                for sentence in sentences:
                    if not sentence.strip():
                        continue
                    
                    sentence = sentence.strip() + "。"  # 添加句号
                    
                    # 如果单个句子就超过最大长度，则按逗号进一步分割
                    if len(sentence) > max_length:
                        clauses = re.split(r'[，,]', sentence)
                        for clause in clauses:
                            if not clause.strip():
                                continue
                            clause = clause.strip()
                            # 如果子句还是太长，则强制按长度分割
                            if len(clause) > max_length:
                                # 强制分割长子句
                                while len(clause) > max_length:
                                    segments.append(clause[:max_length])
                                    clause = clause[max_length:]
                                if clause:
                                    segments.append(clause)
                            else:
                                segments.append(clause)
                    else:
                        # 检查添加当前句子是否会超过最大长度
                        if len(current_segment) + len(sentence) <= max_length:
                            current_segment += sentence
                        else:
                            # 当前段已满，保存并开始新段
                            if current_segment:
                                segments.append(current_segment)
                            current_segment = sentence
                
                # 添加最后一段
                if current_segment:
                    segments.append(current_segment)
                
                # 逐段播放
                for i, segment in enumerate(segments):
                    # 检查是否应该停止播放
                    if not self.tts_playing or self.state == "stopped":
                        print("⏹️ TTS播放被中断")
                        break
                    
                    print(f"[DEBUG] 播放TTS文本段 {i+1}/{len(segments)}: {segment}")
                    result = self.audio_processor.audio_client.TtsMaker(segment, speaker_id)
                    if result != 0:
                        print(f"⚠️  TTS播放返回错误码: {result}")
                        # 添加短暂延迟再重试一次
                        time.sleep(0.5)
                        result = self.audio_processor.audio_client.TtsMaker(segment, speaker_id)
                        if result != 0:
                            print(f"⚠️  TTS播放重试失败，错误码: {result}")
                    # 等待当前段播放完成，但定期检查中断信号
                    wait_time = max(1.0, len(segment) * 0.2)  # 每字符0.2秒
                    elapsed = 0
                    check_interval = 1.0  # 每秒检查一次中断信号
                    while elapsed < wait_time and self.tts_playing and self.state != "stopped":
                        sleep_time = min(check_interval, wait_time - elapsed)
                        time.sleep(sleep_time)
                        elapsed += sleep_time
                
                # 如果循环正常结束，说明所有段都播放完成
                if self.tts_playing and self.state != "stopped":
                    print("✅ TTS文本分段播放完成")
            else:
                # 文本长度适中，直接播放
                # 检查是否应该停止播放
                if not self.tts_playing or self.state == "stopped":
                    print("⏹️ TTS播放被中断")
                    return
                
                result = self.audio_processor.audio_client.TtsMaker(text, speaker_id)
                if result != 0:
                    print(f"⚠️  TTS播放返回错误码: {result}")
                    # 添加短暂延迟再重试一次
                    time.sleep(0.5)
                    result = self.audio_processor.audio_client.TtsMaker(text, speaker_id)
                    if result != 0:
                        print(f"⚠️  TTS播放重试失败，错误码: {result}")
                # 等待播放完成，但定期检查中断信号
                wait_time = max(2.0, min(15.0, len(text) * 0.2))  # 每字符0.2秒，最多等待15秒
                print(f"[DEBUG] TTS文本长度: {len(text)}, 等待时间: {wait_time:.1f}秒")
                elapsed = 0
                check_interval = 1.0  # 每秒检查一次中断信号
                while elapsed < wait_time and self.tts_playing and self.state != "stopped":
                    sleep_time = min(check_interval, wait_time - elapsed)
                    time.sleep(sleep_time)
                    elapsed += sleep_time
                
                # 如果正常结束等待，说明播放完成
                if self.tts_playing and self.state != "stopped":
                    print("✅ TTS文本播放完成")
            return True
        except Exception as e:
            print(f"❌ TTS播放失败: {e}")
            return False

    def _play_tts_only(self, text, speaker_id=0):
        """
        仅播放TTS文本（无动作）
        
        Args:
            text: 要播放的TTS文本
            speaker_id: 说话人ID
        """
        try:
            # 设置TTS播放状态为True
            self.tts_playing = True
            
            # 使用_play_tts_with_wait方法处理TTS播放
            self._play_tts_with_wait(text, speaker_id)
        except Exception as e:
            print(f"❌ TTS播放失败: {e}")
        finally:
            # 确保TTS播放状态被重置
            self.tts_playing = False

    def _play_tts_with_action(self, text, action_dir_name, speaker_id=0):
        """
        播放TTS文本并同时播放对应目录下的动作
        
        Args:
            text: 要播放的TTS文本
            action_dir_name: 动作目录名称（如"start_a"、"start_b"等）
            speaker_id: 说话人ID
        """
        try:
            print(f"🔊 开始播放TTS文本并同时播放{action_dir_name}目录下的动作")
            
            # 将音量设置为最小
            self._set_volume_to_minimum()
            
            # 设置TTS播放状态为True
            self.tts_playing = True
            
            # 播放开始预设动作（high five仅用于start_a）
            start_action_executed = False
            if self.arm_action_client and self.action_map and action_dir_name == "start_a":
                print("💪 播放开始预设动作: high five")
                self.arm_action_client.ExecuteAction(self.action_map.get("high five"))
                start_action_executed = True
                # 等待动作执行完成
                time.sleep(2)
                # 释放手臂，准备播放自定义动作
                self.arm_action_client.ExecuteAction(self.action_map.get("release arm"))
                time.sleep(0.5)
            
            # 构建动作目录路径 - 修复路径构建逻辑，确保在action目录下查找子目录
            start_action_dir = os.path.join(self.action_dir, action_dir_name)
            print(f"📁 查找动作文件目录: {start_action_dir}")
            
            # 检查动作目录是否存在
            if not os.path.exists(start_action_dir):
                print(f"⚠️  动作目录 {start_action_dir} 不存在，仅播放TTS文本")
                # 仅播放TTS文本
                self._play_tts_only(text, speaker_id)
                self.tts_playing = False
                # 播放结束预设动作
                if self.arm_action_client and self.action_map:
                    try:
                        if action_dir_name == "start_a" and start_action_executed:
                            print("💪 播放结束预设动作: right heart")
                            # 添加3秒延迟再播放结束动作
                            time.sleep(3)
                            self.arm_action_client.ExecuteAction(self.action_map.get("right heart"))
                            time.sleep(2)
                            # 释放手臂
                            self.arm_action_client.ExecuteAction(self.action_map.get("release arm"))
                        elif action_dir_name == "start_x":
                            print("💪 播放结束预设动作: high wave")
                            # 添加3秒延迟再播放结束动作
                            time.sleep(4)
                            self.arm_action_client.ExecuteAction(self.action_map.get("high wave"))
                            time.sleep(2)
                            # 释放手臂
                            self.arm_action_client.ExecuteAction(self.action_map.get("release arm"))
                    except Exception as e:
                        print(f"⚠️ 释放手臂时出错: {e}")
                return
            
            # 加载动作文件
            action_files = glob.glob(os.path.join(start_action_dir, "*.npz"))
            if not action_files:
                print(f"⚠️  动作目录 {start_action_dir} 中未找到动作文件，仅播放TTS文本")
                # 仅播放TTS文本
                self._play_tts_only(text, speaker_id)
                self.tts_playing = False
                # 播放结束预设动作
                if self.arm_action_client and self.action_map:
                    try:
                        if action_dir_name == "start_a":
                            print("💪 播放结束预设动作: right heart")
                            # 添加3秒延迟再播放结束动作
                            time.sleep(3)
                            self.arm_action_client.ExecuteAction(self.action_map.get("right heart"))
                            time.sleep(2)
                            # 释放手臂
                            self.arm_action_client.ExecuteAction(self.action_map.get("release arm"))
                        elif action_dir_name == "start_x":
                            print("💪 播放结束预设动作: high wave")
                            # 添加3秒延迟再播放结束动作
                            time.sleep(3)
                            self.arm_action_client.ExecuteAction(self.action_map.get("high wave"))
                            time.sleep(2)
                            # 释放手臂
                            self.arm_action_client.ExecuteAction(self.action_map.get("release arm"))
                    except Exception as e:
                        print(f"⚠️ 释放手臂时出错: {e}")
                return
            
            # 按文件名排序
            action_files.sort()
            
            # 加载并合并所有动作文件
            all_action_data = []
            fps = 30.0
            
            for npz_file in action_files:
                try:
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
                    
                    # 获取fps（使用第一个文件的fps）
                    if 'fps' in data:
                        raw_fps = data['fps']
                        fps = float(raw_fps.item() if isinstance(raw_fps, np.ndarray) else raw_fps)
                        
                except Exception as e:
                    print(f"❌ 加载动作文件 {npz_file} 失败: {e}")
                    continue
            
            if not all_action_data:
                print(f"⚠️  未成功加载任何动作文件，仅播放TTS文本")
                # 仅播放TTS文本
                self._play_tts_only(text, speaker_id)
                self.tts_playing = False
                # 播放结束预设动作
                if self.arm_action_client and self.action_map:
                    try:
                        if action_dir_name == "start_a" and start_action_executed:
                            print("💪 播放结束预设动作: right heart")
                            self.arm_action_client.ExecuteAction(self.action_map.get("right heart"))
                            time.sleep(2)
                            # 释放手臂
                            self.arm_action_client.ExecuteAction(self.action_map.get("release arm"))
                        elif action_dir_name == "start_x":
                            print("💪 播放结束预设动作: high wave")
                            # 添加3秒延迟再播放结束动作
                            time.sleep(3)
                            self.arm_action_client.ExecuteAction(self.action_map.get("high wave"))
                            time.sleep(2)
                            # 释放手臂
                            self.arm_action_client.ExecuteAction(self.action_map.get("release arm"))
                    except Exception as e:
                        print(f"⚠️ 释放手臂时出错: {e}")
                return
            
            # 合并所有动作数据
            merged_action_data = np.vstack(all_action_data)
            print(f"✅ 成功加载动作序列: {len(action_files)} 个文件，共 {len(merged_action_data)} 帧，fps: {fps:.1f}")
            
            # 保存原始动作数据和设置
            original_action_data = getattr(self, 'action_data', None)
            original_fps = getattr(self, 'fps', None)
            original_dt = getattr(self, 'dt', None)
            original_loop = self.loop
            
            # 设置临时动作数据
            self.action_data = merged_action_data
            self.fps = fps
            self.dt = 1.0 / fps
            
            # 设置循环播放标志
            self.loop = True  # 循环播放以匹配TTS播放时间
            
            # 播放动作（从当前位置平滑过渡到第一帧）
            self.state = "ramp_in"
            self.ramp_start_time = time.time()
            self.current_frame = 0
            
            # 发送启用臂部控制命令
            try:
                self.low_cmd.motor_cmd[G1JointIndex.kArmSdkEnable].q = 1.0
                self.low_cmd.crc = self.crc.Crc(self.low_cmd)
                self.publisher.Write(self.low_cmd)
            except Exception as e:
                print(f"❌ 发送播放命令失败: {e}")
            
            # 在单独的线程中播放TTS文本
            import threading
            def play_tts():
                try:
                    self._play_tts_only(text, speaker_id)
                except Exception as e:
                    print(f"❌ 播放TTS文本失败: {e}")
                finally:
                    # TTS播放完成后设置标志
                    self.tts_playing = False
            
            tts_thread = threading.Thread(target=play_tts)
            tts_thread.daemon = True
            tts_thread.start()
            
            # 循环播放动作直到TTS播放完成或被用户中断
            # 动作播放时长计算（秒）
            action_duration = len(merged_action_data) / fps
            print(f"[DEBUG] 单次动作播放时长: {action_duration:.2f} 秒")
            
            # 循环播放直到TTS播放完成或被用户中断
            while self.tts_playing and self.state != "stopped":
                # 使用update方法来实际播放动作
                start_time = time.time()
                elapsed = 0
                while elapsed < action_duration and self.tts_playing and self.state != "stopped":
                    self.update()
                    time.sleep(0.02)  # 50Hz更新频率
                    elapsed = time.time() - start_time
                
                # 如果TTS仍在播放且动作播放完成，并且未被用户中断，则重置动作播放状态以实现循环
                if self.tts_playing and self.state != "stopped":
                    self.state = "ramp_in"
                    self.ramp_start_time = time.time()
                    self.current_frame = 0
            
            # 检查是否被用户中断（L1+F1）
            if self.state == "stopped":
                print("⏹️ 用户中断播放，停止动作播放")
                # 确保TTS播放也被停止
                self.tts_playing = False
            else:
                # TTS播放完成后停止动作
                print("⏹️ TTS播放完成，停止动作播放")
                # 设置标志以避免播放完成提示音
                self._no_tts_complete = True
                self.stop_play(no_tts=True)  # 不播放TTS提示
                
            # 等待一小段时间确保动作完全停止
            time.sleep(0.5)
            
            # 播放结束预设动作
            if self.arm_action_client and self.action_map:
                try:
                    action_name = None
                    if action_dir_name == "start_a":
                        action_name = "right heart"
                    elif action_dir_name == "start_x":
                        action_name = "high wave"
            
                    if action_name:
                        print(f"💪 播放结束预设动作: {action_name}")
                        # 统一添加3秒延迟
                        time.sleep(3)
                        self.arm_action_client.ExecuteAction(self.action_map.get(action_name))
                        time.sleep(2)
                        # 释放手臂
                        self.arm_action_client.ExecuteAction(self.action_map.get("release arm"))
                except Exception as e:
                    print(f"⚠️ 释放手臂时出错: {e}")
            
            # 恢复原始动作数据和设置
            if original_action_data is not None:
                self.action_data = original_action_data
            if original_fps is not None:
                self.fps = original_fps
            if original_dt is not None:
                self.dt = original_dt
            self.loop = original_loop
            
        except Exception as e:
            print(f"❌ 播放TTS文本和动作时出错: {e}")
        finally:
            # 注意：不在这里释放手臂和禁用臂部控制，因为stop_play()已经处理了这些操作
            # 并且可能正在进行平滑过渡到初始姿态的过程，过早释放会影响平滑性
            
            # 恢复原始音量
            self._restore_original_volume()
            
            # 确保重置TTS播放状态
            self.tts_playing = False
            
            # 任务完成后自动退出程序
            print("✅ TTS和动作播放任务完成，程序即将退出")
            import sys
            import os
            sys.exit(0)