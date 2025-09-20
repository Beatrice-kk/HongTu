#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import time
import json
import numpy as np
import math
import glob
import threading
import subprocess
import shutil
from datetime import datetime
try:
    import rospy
    from std_srvs.srv import Trigger, TriggerResponse
    from std_msgs.msg import String as RosString
except Exception:
    rospy = None

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../../"))
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

# 导入新的音频处理模块
try:
    from g1_audio_processor import G1AudioProcessor
except ImportError:
    # 如果直接运行脚本，可能需要添加当前目录到路径
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from g1_audio_processor import G1AudioProcessor


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
# 遥控器解析
# -------------------------------
class RemoteController:
    def __init__(self):
        self.L1 = 0
        self.L2 = 0
        self.R1 = 0
        self.R2 = 0
        self.A = 0
        self.B = 0
        self.X = 0
        self.Y = 0
        self.Up = 0
        self.Down = 0
        self.Left = 0
        self.Right = 0
        self.F1 = 0
        self.Start = 0
        self.Select = 0
        # 添加按键状态的上一次值，用于检测变化
        self._last_values = {}

    def parse(self, data):
        # 根据xKeySwitchUnion结构解析按键
        # 第一个字节 [2] 包含 R1, L1, start, select, R2, L2, F1, F2
        self.R1 = (data[2] >> 0) & 1
        self.L1 = (data[2] >> 1) & 1
        self.Start = (data[2] >> 2) & 1
        self.Select = (data[2] >> 3) & 1
        self.R2 = (data[2] >> 4) & 1
        self.L2 = (data[2] >> 5) & 1
        self.F1 = (data[2] >> 6) & 1
        # F2 = (data[2] >> 7) & 1
        
        # 第二个字节 [3] 包含 A, B, X, Y, up, right, down, left
        self.A = (data[3] >> 0) & 1
        self.B = (data[3] >> 1) & 1
        self.X = (data[3] >> 2) & 1
        self.Y = (data[3] >> 3) & 1
        self.Up = (data[3] >> 4) & 1
        self.Right = (data[3] >> 5) & 1
        self.Down = (data[3] >> 6) & 1
        self.Left = (data[3] >> 7) & 1
    
    def is_pressed_once(self, key_name):
        """
        检查按键是否是刚刚按下的（按键边缘检测）
        
        Args:
            key_name: 按键名称
            
        Returns:
            bool: 如果按键是刚刚按下的返回True，否则返回False
        """
        current_value = getattr(self, key_name, 0)
        last_value = self._last_values.get(key_name, 0)
        self._last_values[key_name] = current_value
        return current_value and not last_value
    
    def get_combo_once(self, key1, key2):
        """
        检查组合键是否是刚刚按下的
        
        Args:
            key1: 第一个按键名称
            key2: 第二个按键名称
            
        Returns:
            bool: 如果组合键是刚刚按下的返回True，否则返回False
        """
        current_pressed = getattr(self, key1, 0) and getattr(self, key2, 0)
        last_pressed = self._last_values.get(f"{key1}+{key2}", False)
        self._last_values[f"{key1}+{key2}"] = current_pressed
        return current_pressed and not last_pressed


# -------------------------------
# 语音识别处理
# -------------------------------

# -------------------------------
# 动作播放器（完整播放 + 平滑归零）
# -------------------------------
class G1ActionPlayer:
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
            'A': "亲爱的游客朋友们，大家好！欢迎光临国家5A级旅游景区——沙家浜！这里不仅是一座风光秀美的江南水乡，更是一方承载着“芦荡火种、鱼水情深”红色记忆的圣地。在这里，您可以走进沙家浜革命历史纪念馆，聆听那段可歌可泣的英雄故事，感受新四军与人民群众并肩作战的烽火岁月；也可以漫步芦苇荡间，乘一叶轻舟穿梭于碧波芦海，享受水乡的宁静野趣，领略独特的湿地风情；还可以来到横泾老街，漫步于上世纪三四十年代风貌的街巷，观看精彩的民俗表演，品尝地道的水乡美食，沉浸式体验淳朴悠然的江南民俗。沙家浜，是一幅自然与人文交织的画卷，更是一段值得用心感受的历史。愿您在这里度过一段充实而美好的时光！",
            'B': "各位朋友，大家好。在江南水乡沙家浜，曾镌刻下一段军民同心、共抗敌寇的红色记忆。这里有指导员郭建光的壮志凌云，有阿庆嫂的机智沉着，有沙奶奶的慈爱坚毅，也有与敌人周旋的惊心动魄。接下来，让我们循着京剧《沙家浜》的经典旋律，一同穿越烽火岁月，重温那段充满斗争智慧与深厚情谊的历史！",
            'C': "各位朋友，经典的唱腔余韵悠长，烽火里的故事依旧动人。我们刚刚一同重温了郭建光的壮志、沙奶奶的坚韧，也深深记住了阿庆嫂“垒起七星灶”的过人智慧，更读懂了那份跨越岁月的军民鱼水情。本场沙家浜京剧选段演出到此圆满结束，感谢您的驻足与陪伴，我们下次再会！",
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
        self.move_to_initial_duration = 2.5  # 回到初始姿态的时间
        self.ramp_start_time = None
        self.start_time = None
        self.current_frame = 0

        # 当前反馈 (初始化为None，表示尚未获取到反馈)
        self.current_pose = None
        
        # 初始姿态（固定为零位）
        self.initial_pose = np.zeros(15, dtype=np.float32)
        print(f"🔧 初始姿态设置为零位: {self.initial_pose[:3]}")
        
        # 程序启动时的姿态
        self.startup_pose = None

        # 控制参数 - 优化以减少电机抖动
        self.base_kp_waist = 65.0   # 从100.0降低以减少抖动
        self.base_kp_arm = 40.0     # 从60.0降低以减少抖动

        self.base_kd_waist = 7    # 从5.0增加以提高阻尼
        self.base_kd_arm = 5      # 从3.0增加以提高阻尼
        
        # 动作幅度缩放因子 (减小动作幅度以提高平滑性和平衡性)
        self.action_scale_factor = 0.9  # 缩放到70%的动作幅度
        
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
        import threading
        self.audio_playback_stop_event = threading.Event()
        self.audio_playback_active = False  # 音频播放状态标志
        
        # 音频处理器
        try:
            self.audio_processor = G1AudioProcessor()
            print("🔊 音频处理器初始化成功")
        except Exception as e:
            print(f"❌ 音频处理器初始化失败: {e}")
            # 创建一个简化版本的音频处理器，确保基本功能可用
            class DummyAudioProcessor:
                def __init__(self):
                    try:
                        from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
                        self.audio_client = AudioClient()
                        self.audio_client.SetTimeout(5.0)
                        self.audio_client.Init()
                        print("🔊 音频客户端初始化成功")
                    except Exception as e:
                        print(f"❌ 音频客户端初始化失败: {e}")
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
            print("💪 手臂动作客户端初始化成功")
        except Exception as e:
            print(f"❌ 手臂动作客户端初始化失败: {e}")
            self.arm_action_client = None
            self.action_map = None

        # 外部进程句柄（用于避免重复启动）
        self._fastlio_proc = None
        self._fastlio_started_at = None

        self.load_actions()
        self.setup_publisher()
    
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
                        
                        print(f"✅ 加载动作: {direction_key} (来自 {direction_dir}/) -> '{clean_name}' | 帧数: {len(merged_action_data)} | fps: {fps:.1f} | 文件数: {len(npz_file_group)}")
                        if audio_file:
                            print(f"🎵 关联音频文件: {audio_file}")
                    except Exception as e:
                        print(f"❌ 加载动作文件组 {clean_name} 失败: {e}")
        
        if not self.actions:
            print("⚠️  未找到任何动作文件")
        else:
            print(f"✅ 成功加载 {len(self.actions)} 个动作")
            # 显示加载的动作详情
            for direction_key, action in self.actions.items():
                print(f"  🎭 {direction_key}: {action['name']} (来自 {action['source_dir']}/)")
                if action.get('audio_file'):
                    print(f"     🎵 音频: {action['audio_file']}")
                if 'original_files' in action and len(action['original_files']) > 1:
                    print(f"     📂 分割文件: {len(action['original_files'])} 个")
    
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
            action_data: 原始动作数据 (N, 15)
            max_angle_delta: 允许的最大角度变化（弧度），调小该值以适应咏春拳等精细动作
            min_fps: 最小帧率，用于确定插帧数量
            
        Returns:
            平滑处理后的动作数据
        """
        if len(action_data) < 2:
            return action_data
            
        smoothed_data = []
        
        # 针对剧烈动作（如永春、刺探）降低角度变化阈值，提高插帧密度
        # 检查是否为剧烈动作（通过整体动作幅度判断）
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
                # 计算需要插入的帧数，根据角度变化大小确定插帧数量
                # 使用更精确的计算方法，考虑动作剧烈程度
                base_frames = max_angle_delta / 0.15  # 基础帧数
                dynamic_frames = max_delta / max_angle_delta  # 根据实际变化调整
                num_insert_frames = int(np.ceil(dynamic_frames * base_frames))
                num_insert_frames = max(1, min(num_insert_frames, 30))  # 增加最大插帧数到30以适应剧烈动作
                
                print(f"⚠️  检测到剧烈运动: 帧 {i}-{i+1}, 最大角度变化 {max_delta:.3f} rad, 插入 {num_insert_frames} 帧")
                
                # 使用更高阶的插值方法（三次样条插值效果更好）
                for j in range(1, num_insert_frames + 1):
                    # 使用缓入缓出函数，使动作变化更平滑
                    t = j / (num_insert_frames + 1)
                    # 使用更平滑的插值函数
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
    
    def play_action(self, direction, speed=1.0):
        """播放指定方向的动作
        
        Args:
            direction: 动作方向
            speed: 播放速度倍数，默认为1.0（正常速度）
                  大于1.0表示加速播放，小于1.0表示减速播放
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
            
        # 配置参数
        config = {
            'ramp_in_duration': 1.2,  # 调整平滑进入时间，在响应速度和平滑度之间取得平衡
            'led_color': (0, 255, 0),  # 绿色灯光
            'tts_message': f"我给大家表演{self.actions[direction]['name']}"
        }
        
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
        
        # 播放开始提示音并移除灯光控制
        tts_start_time = None
        try:
            # 添加防重复机制
            if not hasattr(self, '_last_play_tts_time'):
                self._last_play_tts_time = 0
            current_time = time.time()
            if current_time - self._last_play_tts_time > 3.0:  # 至少间隔1秒
                if self.audio_processor and hasattr(self.audio_processor, 'audio_client'):
                    self.audio_processor.audio_client.TtsMaker(config['tts_message'], 0)
                    tts_start_time = current_time
                self._last_play_tts_time = current_time
        except Exception as e:
            print(f"❌ 播放开始提示失败: {e}")
        
        # 等待TTS播报完成（估计时间约3秒）
        if tts_start_time is not None:
            tts_wait_time = 5.0  # 等待3秒确保TTS播报完成
            time_to_wait = tts_wait_time - (time.time() - tts_start_time)
            if time_to_wait > 0:
                time.sleep(time_to_wait)
            print("✅ TTS播报已完成")
        
        # 添加额外延迟确保TTS完全结束
        time.sleep(0.5)
        
        # 如果有关联的音频文件，则播放音频
        self.audio_duration = None  # 重置音频时长
        if action.get('audio_file'):
            self._play_associated_audio(action)
        
        self.state = "ramp_in"  # 保持ramp_in状态以确保平滑开始
        self.ramp_start_time = time.time()
        self.start_time = self.ramp_start_time  # 将start_time设置为ramp_in开始时间，确保总时间计算正确
        self.current_frame = 0
        try:
            self.low_cmd.motor_cmd[G1JointIndex.kArmSdkEnable].q = 1.0
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.publisher.Write(self.low_cmd)
        except Exception as e:
            print(f"❌ 发送播放命令失败: {e}")
            
        return True

    def _play_associated_audio(self, action):
        """
        播放与动作关联的音频文件
        
        Args:
            action: 动作数据字典
        """
        try:
            print(f"🎵 播放关联音频: {action['audio_file']}")
            # 停止可能正在播放的任何音频
            if self.audio_processor and hasattr(self.audio_processor, 'audio_client'):
                self.audio_processor.audio_client.PlayStop("g1_client")
            
            # 重置音频播放控制事件
            self.audio_playback_stop_event.clear()
            self.audio_playback_active = True  # 在开始播放前设置为活跃状态
            
            # 导入音频处理函数
            from wav import read_wav, get_wav_duration
            
            # 获取音频时长
            audio_duration = get_wav_duration(action['audio_file'])
            if audio_duration > 0:
                self.audio_duration = audio_duration
                print(f"[DEBUG] 音频时长: {audio_duration:.2f} 秒 ({int(audio_duration//60)}分{int(audio_duration%60)}秒)")
            
            # 读取音频文件
            pcm_list, sample_rate, num_channels, is_ok = read_wav(action['audio_file'])
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
        except Exception as e:
            print(f"❌ 播放音频文件失败: {e}")
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

            
        except Exception as e:
            print(f"[ERROR] 音频格式转换失败: {e}")
            return None
    
    def _resample_audio(self, audio_data, original_rate, target_rate):
        """
        使用更高质量的算法进行音频重采样
        
        Args:
            audio_data: 原始音频数据 (numpy array)
            original_rate: 原始采样率
            target_rate: 目标采样率
            
        Returns:
            重采样后的音频数据
        """
        try:
            import numpy as np
            
            original_length = len(audio_data)
            target_length = int(original_length * target_rate / original_rate)
            
            # 如果目标长度与原始长度相同，则直接返回
            if target_length == original_length:
                return audio_data
            
            # 使用更高质量的重采样方法
            if target_length > original_length:
                # 上采样 - 使用三次样条插值
                from scipy.interpolate import interp1d
                original_indices = np.arange(original_length)
                target_indices = np.linspace(0, original_length - 1, target_length)
                
                # 使用三次样条插值进行上采样
                interpolator = interp1d(original_indices, audio_data, kind='cubic', fill_value='extrapolate')
                resampled_data = interpolator(target_indices)
            else:
                # 下采样 - 使用抗混叠滤波和高质量插值
                from scipy import signal
                # 计算抗混叠滤波器
                nyquist = min(original_rate, target_rate) / 2.0
                cutoff = nyquist * 0.9  # 保留90%的频率范围，获得更好的音质
                filter_order = 12  # 增加滤波器阶数以获得更好的滤波效果
                
                # 设计低通滤波器
                sos = signal.butter(filter_order, cutoff, btype='low', fs=original_rate, output='sos')
                filtered_data = signal.sosfilt(sos, audio_data)
                
                # 使用三次样条插值进行下采样（已经滤波过，避免混叠）
                from scipy.interpolate import interp1d
                original_indices = np.arange(original_length)
                target_indices = np.linspace(0, original_length - 1, target_length)
                
                interpolator = interp1d(original_indices, filtered_data, kind='cubic', fill_value='extrapolate')
                resampled_data = interpolator(target_indices)
            
            return resampled_data
            
        except ImportError:
            # 如果scipy不可用，回退到简单的线性插值
            print("[WARNING] scipy不可用，使用简单的线性插值进行重采样")
            return self._simple_resample(audio_data, original_rate, target_rate)
        except Exception as e:
            print(f"[ERROR] 重采样过程中出错: {e}")
            # 出错时回退到简单的线性插值
            return self._simple_resample(audio_data, original_rate, target_rate)
    
    def _simple_resample(self, audio_data, original_rate, target_rate):
        """
        简单的线性插值重采样方法
        
        Args:
            audio_data: 原始音频数据 (numpy array)
            original_rate: 原始采样率
            target_rate: 目标采样率
            
        Returns:
            重采样后的音频数据
        """
        import numpy as np
        original_length = len(audio_data)
        target_length = int(original_length * target_rate / original_rate)
        
        indices = np.linspace(0, original_length - 1, target_length)
        resampled_data = np.interp(indices, np.arange(original_length), audio_data)
        return resampled_data

    def _stop_audio_playback(self):
        """停止音频播放"""
        try:
            print("⏹️ 正在停止音频播放...")
            # 设置停止事件
            self.audio_playback_stop_event.set()
            # 调用音频客户端的停止方法
            if self.audio_processor and hasattr(self.audio_processor, 'audio_client'):
                self.audio_processor.audio_client.PlayStop("g1_client")
            # 更新播放状态
            self.audio_playback_active = False
            print("✅ 音频播放已停止")
        except Exception as e:
            print(f"❌ 停止音频播放时出错: {e}")
        finally:
            # 重置停止事件，为下一次播放做准备
            self.audio_playback_stop_event.clear()
    
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
                pcm_data = bytes(pcm_list)
            
            # 在开始播放前稍作等待，确保系统准备就绪
            time.sleep(0.1)
            
            stream_id = str(int(time.time() * 1000))  # 基于当前时间戳的唯一流ID
            offset = 0
            chunk_index = 0
            total_size = len(pcm_data)
            
            # 计算音频时长（用于同步动作播放）
            bytes_per_sample = 2  # 16位音频每个样本2字节
            sample_rate = 16000   # G1机器人要求的采样率
            total_samples = total_size // bytes_per_sample
            self.audio_duration = total_samples / sample_rate  # 音频时长（秒）
            print(f"[DEBUG] 音频时长: {self.audio_duration:.2f} 秒 ({self.audio_duration//60:.0f}分{self.audio_duration%60:.0f}秒)")
            
            # 使用与官方示例相同的参数
            effective_chunk_size = min(chunk_size, 24000)  # 24000字节 = 0.75秒的16kHz单声道音频
            effective_sleep_time = max(sleep_time, 0.75)   # 与官方示例一致
            
            # 增加重试机制参数
            max_retries = 5  # 最大重试次数
            consecutive_failures = 0  # 连续失败计数
            max_consecutive_failures = 10  # 最大连续失败次数

            while offset < total_size and not self.audio_playback_stop_event.is_set():
                remaining = total_size - offset
                current_chunk_size = min(effective_chunk_size, remaining)
                chunk = pcm_data[offset:offset + current_chunk_size]
                
                # 发送块，带重试机制
                retry_count = 0
                while retry_count <= max_retries:
                    ret_code, _ = client.PlayStream(stream_name, stream_id, chunk)
                    if ret_code == 0:
                        # 成功发送
                        if verbose or chunk_index % 10 == 0:  # 每10个块报告一次
                            print(f"[INFO] Chunk {chunk_index} sent successfully")
                        consecutive_failures = 0  # 重置连续失败计数
                        break
                    else:
                        # 发送失败
                        retry_count += 1
                        consecutive_failures += 1
                        print(f"[ERROR] Failed to send chunk {chunk_index}, return code: {ret_code}, retry {retry_count}/{max_retries}")
                        
                        # 如果连续失败次数过多，则终止播放
                        if consecutive_failures >= max_consecutive_failures:
                            print(f"[ERROR] Too many consecutive failures ({consecutive_failures}), stopping audio playback")
                            return
                            
                        # 如果达到最大重试次数，则移动到下一个块
                        if retry_count > max_retries:
                            break
                            
                        # 等待一段时间再重试
                        time.sleep(0.1 * retry_count)

                offset += current_chunk_size
                chunk_index += 1
                
                # 检查是否需要停止播放
                if self.audio_playback_stop_event.is_set():
                    print("[INFO] Audio playback stopped by user request")
                    break
                    
                # 休眠以避免网络拥塞
                time.sleep(effective_sleep_time)

        except Exception as e:
            print(f"[ERROR] Audio playback failed: {e}")
            import traceback
            traceback.print_exc()
        finally:
            # 播放完成后发送停止命令
            try:
                # 在停止前稍作等待，确保最后的数据包发送完毕
                time.sleep(0.1)
                client.PlayStop(stream_name)
                print("[INFO] Audio playback finished and stopped")
            except Exception as e:
                print(f"[ERROR] Failed to stop audio playback: {e}")
            # 确保在播放完成后更新状态
            self.audio_playback_active = False
    
    def init_to_zero_position(self):
        """
        程序启动后主动执行平滑移动到预设安全位置
        """
        print("🔄 程序启动，开始执行初始化到预设安全位置...")
        
        # 播放初始化提示音并移除灯光控制
        try:
            # 添加防重复机制
            if not hasattr(self, '_last_init_tts_time'):
                self._last_init_tts_time = 0
            current_time = time.time()
            if current_time - self._last_init_tts_time > 3.0:  # 至少间隔1秒
                self.audio_processor.audio_client.TtsMaker("系统启动，正在初始化", 0)
                self._last_init_tts_time = current_time
        except Exception as e:
            print(f"❌ 音频初始化提示失败: {e}")
        
        # 等待获取实际的当前位置反馈
        wait_start = time.time()
        while self.current_pose is None and (time.time() - wait_start) < 5.0:  # 等待最多5秒
            print("⏳ 等待接收实际关节位置反馈...")
            time.sleep(0.1)
            
        if self.current_pose is None:
            # 如果超时仍未收到反馈，不能继续执行初始化
            print("❌ 超时未收到反馈，无法执行初始化")
            try:
                self.audio_processor.audio_client.TtsMaker("初始化失败", 0)
            except Exception as e:
                print(f"❌ 错误提示播放失败: {e}")
            return
            
        print(f"✅ 收到实际关节位置反馈: {self.current_pose[:3]}")
        
        # 定义预设的安全初始姿态
        target_pose = np.zeros(15, dtype=np.float32)
        # 腰部保持零位
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

            # print(f"   过渡进度: {ratio:.2f}, 当前目标: {current_target[:3]}")
            
            # 发送命令，使用较低的刚度确保安全
            self._send_pose(current_target, dq=np.zeros(15), kp_scale=0.3, kd_scale=1.0)
            time.sleep(0.02)  # 50Hz控制频率，保持不变
            
        # 确保最终位置
        self._send_pose(target_pose, dq=np.zeros(15), kp_scale=0.3, kd_scale=1.0)
        time.sleep(0.1)
        
        # 保存这个预设位置作为初始姿态
        self.startup_pose = target_pose.copy()
        print("✅ 初始化到预设安全位置完成")
        print(f"📍 当前位置: {target_pose[:3]}")
        
        # 初始化完成后设置状态为stopped，避免持续发送指令
        self.state = "stopped"
        
        # 初始化完成，移除灯光控制
        try:
            self.audio_processor.audio_client.TtsMaker("初始化完成，系统就绪", 0)
            time.sleep(1)
        except Exception as e:
            print(f"❌ 初始化完成提示失败: {e}")

    def _send_interpolated_frame(self, smooth_ratio, target_idx):
        target_q = self.action_data[target_idx].copy()
        start_q = self.current_pose if self.current_pose is not None else np.zeros(15)
        # 应用动作幅度缩放因子以减小动作幅度但保持时间节奏
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
        smoothing_factor_ramp = 0.2  # 略高的平滑系数用于ramp阶段
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
        kp_scale = 0.5 + 0.1 * smooth_ratio  # 显著降低整体刚度以提高平滑性
        kd_scale = 1.2 + 0.3 * smooth_ratio  # 增加阻尼以减少抖动
        self._send_pose(interp_q, dq=np.zeros(15), kp_scale=kp_scale, kd_scale=kd_scale)

    def _send_frame(self, frame_idx):
        q = self.action_data[frame_idx]
        # 在动作播放过程中使用优化的控制参数，在流畅度和力度之间取得平衡
        # 使用适度的平滑函数来调整控制参数
        progress = frame_idx / len(self.action_data) if len(self.action_data) > 0 else 0
        # 使用标准平滑步进插值，在响应速度和平滑度之间取得平衡
        smooth_factor = progress * progress * (3 - 2 * progress)
        
        # 根据播放进度调整控制参数，平衡流畅度和力度
        kp_scale = 0.8 + 0.2 * smooth_factor  # 保持良好的跟踪性能
        kd_scale = 0.9 + 0.1 * smooth_factor  # 适度的阻尼控制以减少抖动
        
        self._send_pose(q, dq=np.zeros(15), kp_scale=kp_scale, kd_scale=kd_scale)


    def setup_publisher(self):
        self.publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)

        self.publisher.Init()
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        # 不在初始化时发送任何命令，避免强制移动关节

    def _send_pose(self, q, dq=None, kp_scale=1.0, kd_scale=1.0):
        # 限制发送频率以减少CPU使用
        current_time = time.time()
        if not hasattr(self, '_last_send_time'):
            self._last_send_time = 0
            
        # 在动作播放期间提高发送频率到25ms一次 (~40Hz)，其他时候保持40ms
        # 降低频率以减少抖动
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

        # 根据状态调整控制参数，优化以减少电机抖动
        if self.state in ["ramp_in", "playing"]:
            # 动作播放期间使用更适合的控制参数
            # 显著降低Kp值以减少抖动，增加Kd值以提高阻尼
            kp_waist = self.base_kp_waist * kp_scale * 0.7   # 进一步降低腰部Kp值
            kp_arm = self.base_kp_arm * kp_scale * 0.6       # 进一步降低手臂Kp值
            kd_waist = self.base_kd_waist * kd_scale * 1.5   # 增加腰部Kd值
            kd_arm = self.base_kd_arm * kd_scale * 1.8       # 增加手臂Kd值
        else:
            # 其他状态保持平滑控制
            kp_waist = self.base_kp_waist * kp_scale * 0.5   # 显著降低腰部Kp值
            kp_arm = self.base_kp_arm * kp_scale * 0.4       # 显著降低手臂Kp值
            kd_waist = self.base_kd_waist * kd_scale * 1.8   # 显著增加腰部Kd值
            kd_arm = self.base_kd_arm * kd_scale * 2.2       # 显著增加手臂Kd值

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

    def stop_play(self, no_tts=False):
        # 只有在播放状态才进入回到初始姿态的流程
        if self.state in ["playing", "ramp_in"]:
            if not no_tts:
                print(f"⏹️ 动作结束，进入平滑回到初始姿态流程")
                # 立即播放结束提示音，不等待，添加防重复机制
                try:
                    # 添加防重复机制
                    if not hasattr(self, '_last_stop_tts_time'):
                        self._last_stop_tts_time = 0
                    current_time = time.time()
                    if current_time - self._last_stop_tts_time > 3.0:  # 至少间隔1秒
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
                    print(f"❌ 播放结束提示失败: {e}")
            else:
                print(f"⏹️ 动作结束，进入平滑回到初始姿态流程（无TTS提示）")
            
            # 停止音频播放
            self._stop_audio_playback()
            
            self.state = "move_to_initial"
            self.ramp_start_time = time.time()
            # 记录当前实际位置作为过渡的起点
            if self.current_pose is not None:
                self.transition_start_pose = self.current_pose.copy()
            else:
                # 如果没有当前位置反馈，使用初始姿态
                self.transition_start_pose = self.startup_pose if hasattr(self, 'startup_pose') and self.startup_pose is not None else self.initial_pose
        elif self.state == "move_to_initial":
            # 如果已经在回到初始姿态的过程中，直接完成
            # 主动释放手臂
            if self.arm_action_client and self.action_map:
                try:
                    self.arm_action_client.ExecuteAction(self.action_map.get("release arm"))
                    print("✅ 手臂已释放")
                except Exception as e:
                    print(f"⚠️ 释放手臂时出错: {e}")
            
            self.state = "stopped"
            print("✅ 状态已设置为 stopped")

    def _init_audio_sync(self):
        """初始化音频同步相关属性"""
        self.audio_playback_active = False
        self.audio_playback_stop_event = threading.Event()
        self._last_audio_check_time = 0
        self._audio_check_interval = 0.1  # 每100ms检查一次音频状态
        
    def _start_audio_playback(self, audio_file):
        """启动音频播放并开始同步检测"""
        try:
            if not self.audio_processor or not hasattr(self.audio_processor, 'audio_client'):
                return False
                
            # 初始化音频同步属性
            self._init_audio_sync()
            
            # 启动音频播放
            result = self.audio_processor.audio_client.PlayWavFile(audio_file, 0)  # 0表示不循环播放
            if result == 0:
                self.audio_playback_active = True
                self.audio_playback_stop_event.clear()
                print(f"🔊 开始播放音频文件: {audio_file}")
                return True
            else:
                print(f"❌ 音频播放启动失败: {audio_file}")
                return False
        except Exception as e:
            print(f"❌ 启动音频播放异常: {e}")
            return False
            
    def _stop_audio_playback(self):
        """停止音频播放"""
        try:
            if self.audio_playback_active:
                # 设置停止事件，通知音频播放线程停止
                self.audio_playback_stop_event.set()
                
                # 停止音频播放
                if self.audio_processor and hasattr(self.audio_processor, 'audio_client'):
                    self.audio_processor.audio_client.PlayStop("g1_client")
                
                self.audio_playback_active = False
                print("🔇 音频播放已停止")
        except Exception as e:
            print(f"❌ 停止音频播放失败: {e}")
            
    def _check_audio_finished(self):
        """检查音频是否播放完成"""
        if not self.audio_playback_active:
            return True
            
        # 如果已经过了停止事件，认为音频播放完成
        if self.audio_playback_stop_event.is_set():
            return True
            
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
                    print(f"[DEBUG] 播放TTS文本段 {i+1}/{len(segments)}: {segment}")
                    result = self.audio_processor.audio_client.TtsMaker(segment, speaker_id)
                    if result != 0:
                        print(f"⚠️  TTS播放返回错误码: {result}")
                        # 添加短暂延迟再重试一次
                        time.sleep(0.5)
                        result = self.audio_processor.audio_client.TtsMaker(segment, speaker_id)
                        if result != 0:
                            print(f"⚠️  TTS播放重试失败，错误码: {result}")
                    # 等待当前段播放完成
                    wait_time = max(1.0, len(segment) * 0.2)  # 每字符0.2秒
                    time.sleep(wait_time)
            else:
                # 文本长度适中，直接播放
                result = self.audio_processor.audio_client.TtsMaker(text, speaker_id)
                if result != 0:
                    print(f"⚠️  TTS播放返回错误码: {result}")
                    # 添加短暂延迟再重试一次
                    time.sleep(0.5)
                    result = self.audio_processor.audio_client.TtsMaker(text, speaker_id)
                    if result != 0:
                        print(f"⚠️  TTS播放重试失败，错误码: {result}")
                # 等待播放完成
                wait_time = max(2.0, min(15.0, len(text) * 0.2))  # 每字符0.2秒，最多等待15秒
                print(f"[DEBUG] TTS文本长度: {len(text)}, 等待时间: {wait_time:.1f}秒")
                time.sleep(wait_time)
            return True
        except Exception as e:
            print(f"❌ TTS播放失败: {e}")
            return False

            
        # 定期检查音频播放状态
        current_time = time.time()
        if current_time - self._last_audio_check_time >= self._audio_check_interval:
            self._last_audio_check_time = current_time
            
            try:
                # 查询音频播放状态
                if self.audio_processor and hasattr(self.audio_processor, 'audio_client'):
                    status = self.audio_processor.audio_client.GetPlayStatus()
                    # 如果状态不是播放中，认为播放完成
                    if status != 1:  # 假设1表示播放中
                        print(f"🔍 音频播放状态: {status}")
                        return True
            except Exception as e:
                print(f"❌ 检查音频状态失败: {e}")
                return True
                
        return False

    def toggle_pause(self):
        if self.state == "playing":
            self.stop_play()
        elif self.state in ["stopped", "soft_hold", "soft_hold_zero"]:
            # 这里可以添加恢复播放的逻辑
            pass
            
    def _play_tts_only(self, text, speaker_id=0):
        """
        仅播放TTS文本（不播放动作）
        
        Args:
            text: 要播放的文本
            speaker_id: 说话人ID
        """
        try:
            # 设置TTS播放状态为True
            self.tts_playing = True
            
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
                    print(f"[DEBUG] 播放TTS文本段 {i+1}/{len(segments)}: {segment}")
                    result = self.audio_processor.audio_client.TtsMaker(segment, speaker_id)
                    if result != 0:
                        print(f"⚠️  TTS播放返回错误码: {result}")
                        # 添加短暂延迟再重试一次
                        time.sleep(0.5)
                        result = self.audio_processor.audio_client.TtsMaker(segment, speaker_id)
                        if result != 0:
                            print(f"⚠️  TTS播放重试失败，错误码: {result}")
                    # 等待当前段播放完成
                    wait_time = max(1.0, len(segment) * 0.2)  # 每字符0.2秒
                    time.sleep(wait_time)
            else:
                # 文本长度适中，直接播放
                result = self.audio_processor.audio_client.TtsMaker(text, speaker_id)
                if result != 0:
                    print(f"⚠️  TTS播放返回错误码: {result}")
                    # 添加短暂延迟再重试一次
                    time.sleep(0.5)
                    result = self.audio_processor.audio_client.TtsMaker(text, speaker_id)
                    if result != 0:
                        print(f"⚠️  TTS播放重试失败，错误码: {result}")
                # 等待播放完成
                wait_time = max(2.0, min(15.0, len(text) * 0.2))  # 每字符0.2秒，最多等待15秒
                print(f"[DEBUG] TTS文本长度: {len(text)}, 等待时间: {wait_time:.1f}秒")
                time.sleep(wait_time)
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
                        if action_dir_name == "start_a":
                            print("💪 播放结束预设动作: right heart")
                            self.arm_action_client.ExecuteAction(self.action_map.get("right heart"))
                            time.sleep(2)
                            # 释放手臂
                            self.arm_action_client.ExecuteAction(self.action_map.get("release arm"))
                        elif action_dir_name == "start_x":
                            print("💪 播放结束预设动作: high wave")
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
                            self.arm_action_client.ExecuteAction(self.action_map.get("right heart"))
                            time.sleep(2)
                            # 释放手臂
                            self.arm_action_client.ExecuteAction(self.action_map.get("release arm"))
                        elif action_dir_name == "start_x":
                            print("💪 播放结束预设动作: high wave")
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
                        if action_dir_name == "start_a":
                            print("💪 播放结束预设动作: right heart")
                            self.arm_action_client.ExecuteAction(self.action_map.get("right heart"))
                            time.sleep(2)
                            # 释放手臂
                            self.arm_action_client.ExecuteAction(self.action_map.get("release arm"))
                        elif action_dir_name == "start_x":
                            print("💪 播放结束预设动作: high wave")
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
                    if action_dir_name == "start_a" and start_action_executed:
                        print("💪 播放结束预设动作: right heart")
                        self.arm_action_client.ExecuteAction(self.action_map.get("right heart"))
                        time.sleep(2)
                        # 释放手臂
                        self.arm_action_client.ExecuteAction(self.action_map.get("release arm"))
                    elif action_dir_name == "start_x":
                        print("💪 播放结束预设动作: high wave")
                        self.arm_action_client.ExecuteAction(self.action_map.get("high wave"))
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
            # 确保在所有情况下都释放手臂
            if self.arm_action_client and self.action_map:
                try:
                    self.arm_action_client.ExecuteAction(self.action_map.get("release arm"))
                    print("✅ 手臂已释放")
                except Exception as e:
                    print(f"⚠️ 释放手臂时出错: {e}")
            # 确保重置TTS播放状态
            self.tts_playing = False
    def update_low_frequency(self):
        """
        低频更新函数，用于在功能未激活时减少CPU占用
        """
        # 限制update_low_frequency函数的执行频率
        current_time = time.time()
        
        # 使用实例属性来存储上次调用时间，避免每次调用都检查hasattr
        if not hasattr(self, '_last_low_freq_update_call'):
            self._last_low_freq_update_call = 0
            
        # 限制update_low_frequency调用频率为200ms一次，显著降低CPU使用率
        if current_time - self._last_low_freq_update_call < 0.2:
            return
            
        self._last_low_freq_update_call = current_time
        
        # 如果尚未获取到当前位置反馈，使用零位作为默认位置
        if self.current_pose is None:
            self.current_pose = np.zeros(15, dtype=np.float32)
            return

        # 在停止状态下，只有在功能激活时才发送保持初始姿态的命令，防止干扰遥控器控制
        if self.state == "stopped" and self.function_activated:
            # 持续发送初始姿态命令以保持位置，但降低频率
            hold_pose = self.startup_pose if hasattr(self, 'startup_pose') and self.startup_pose is not None else self.initial_pose
            self._send_pose(hold_pose, dq=np.zeros(15), kp_scale=0.2, kd_scale=1.0)
    def update(self):
        # 限制update函数的执行频率
        current_time = time.time()
        
        # 使用实例属性来存储上次调用时间，避免每次调用都检查hasattr
        if not hasattr(self, '_last_update_call'):
            self._last_update_call = 0
            
        # 限制update调用频率为50ms一次，降低频率以减少抖动
        if current_time - self._last_update_call < 0.05:  # 从0.04增加到0.05（20Hz）
            return
            
        self._last_update_call = current_time
        
        # 在停止状态下不发送任何控制指令，让遥控器正常工作
        if self.state == "stopped":
            return
            
        # 如果尚未获取到当前位置反馈，使用零位作为默认位置
        if self.current_pose is None:
            self.current_pose = np.zeros(15, dtype=np.float32)

        # 只有在非停止状态下才执行时间相关的状态更新
        t = time.time()

        # -------------------------------
        # 2. playing: 精确播放，播完再退出
        # -------------------------------
        if self.state == "playing":
            elapsed = t - self.start_time
            
            # 确定总时长：优先使用音频时长，否则使用动作帧数计算
            if hasattr(self, 'audio_duration') and self.audio_duration is not None and self.audio_duration > 0:
                total_duration = self.audio_duration
                print(f"[DEBUG] 使用音频时长: {total_duration:.2f} 秒")
            else:
                total_duration = len(self.action_data) * self.dt  # 基于动作帧数计算
                print(f"[DEBUG] 使用动作时长: {total_duration:.2f} 秒")
            
            # 检查动作是否播放完成或者音频是否播放完成
            action_finished = elapsed >= total_duration
            
            # 增强音频播放状态检测 - 检查音频播放线程是否还活跃
            audio_finished = not self.audio_playback_active

            # 如果动作和音频都播放完成，或者超过最大播放时间，则停止播放
            max_duration = max(total_duration * 1.1, 10.0)  # 略微增加最大时长，防止提前结束
            if (action_finished and audio_finished) or (elapsed >= max_duration):
                # 检查是否是TTS播放模式下的循环播放
                if self.tts_playing and self.loop:
                    # 如果TTS仍在播放且设置了循环，则重置播放时间以实现循环
                    print("🔄 TTS仍在播放，重置动作播放时间以实现循环")
                    self.start_time = t
                    self.current_frame = 0
                else:
                    if elapsed >= max_duration:
                        print(f"⏰ 播放超时，强制结束")
                    else:
                        print(f"🎬 动作播放完毕（{len(self.action_data)} 帧），进入退出流程")
                    
                    # 立即播放完成提示音，添加防重复机制
                    try:
                        # 添加防重复机制
                        if not hasattr(self, '_last_finish_tts_time'):
                            self._last_finish_tts_time = 0
                        current_time = time.time()
                        if current_time - self._last_finish_tts_time > 3.0:  # 至少间隔1秒
                            if self.current_action:
                                if self.audio_processor and hasattr(self.audio_processor, 'audio_client'):
                                    self.audio_processor.audio_client.TtsMaker(f"动作{self.current_action['name']}播放完毕，正在回到初始位置", 0)
                            else:
                                if self.audio_processor and hasattr(self.audio_processor, 'audio_client'):
                                    self.audio_processor.audio_client.TtsMaker("动作播放完毕，正在回到初始位置", 0)
                            self._last_finish_tts_time = current_time
                    except Exception as e:
                        print(f"❌ 播放完成提示失败: {e}")
                    self.stop_play()
            else:
                # 动作和音频仍在播放中
                # 精确计算目标帧索引，确保按照原始动作数据播放
                # 确保target_frame不会超出范围
                target_frame = max(0, min(int(elapsed / self.dt), len(self.action_data) - 1))
                self.current_frame = target_frame
                self._send_frame(target_frame)
                
                # 定期报告播放进度
                if not hasattr(self, '_last_progress_report') or (t - self._last_progress_report) >= 1.0:
                    progress = min(elapsed / total_duration, 1.0) if total_duration > 0 else 0
                    print(f"🎵 播放进度: {progress:.1%} ({elapsed:.1f}/{total_duration:.1f}s)")
                    self._last_progress_report = t
            return

        # -------------------------------
        # 1. ramp_in: 当前 → 第一帧（cosine）
        # -------------------------------
        if self.state == "ramp_in":
            elapsed = t - self.ramp_start_time
            ratio = min(elapsed / self.ramp_in_duration, 1.0)
            # 使用标准平滑步进插值，在响应速度和平滑度之间取得平衡
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
        # 3. move_to_initial: 回到初始姿态（程序启动时的姿态）
        # -------------------------------
        if self.state == "move_to_initial":
            elapsed = t - self.ramp_start_time
            duration = self.move_to_initial_duration  # 使用实例属性
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
            
            # 发送插值位置命令，使用优化的控制参数
            # 显著增加阻尼系数以提高平滑性
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
                
                # 播放完成提示音并移除灯光控制（仅在需要时播放）
                if not hasattr(self, '_no_tts_complete') or not self._no_tts_complete:
                    try:
                        # 移除灯光控制: self.audio_processor.audio_client.LedControl(0, 255, 0)  # 绿色灯光
                        if not hasattr(self, '_last_complete_tts_time'):
                            self._last_complete_tts_time = 0
                        current_time = time.time()
                        if current_time - self._last_complete_tts_time > 3.0:  # 至少间隔1秒
                            if self.current_action:
                                self.audio_processor.audio_client.TtsMaker(f"已回到初始位置，{self.current_action['name']}动作完成", 0)
                            else:
                                self.audio_processor.audio_client.TtsMaker("已回到初始位置", 0)
                            self._last_complete_tts_time = current_time
                    except Exception as e:
                        print(f"❌ 播放完成提示失败: {e}")
                else:
                    # 重置标志
                    self._no_tts_complete = False
                
                # 主动释放手臂
                if self.arm_action_client and self.action_map:
                    try:
                        self.arm_action_client.ExecuteAction(self.action_map.get("release arm"))
                        print("✅ 手臂已释放")
                    except Exception as e:
                        print(f"⚠️ 释放手臂时出错: {e}")
                
                # 清空当前动作
                self.current_action = None
                # 清除平滑姿态缓存
                if hasattr(self, 'smoothed_pose'):
                    delattr(self, 'smoothed_pose')
                # 清除ramp阶段的平滑姿态缓存
                if hasattr(self, 'ramp_smoothed_pose'):
                    delattr(self, 'ramp_smoothed_pose')
            return

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
                )
                self._fastlio_started_at = time.time()
        except Exception as e:
            print(f"[fastlio] 启动失败: {e}")
            self._fastlio_started_at = None

    def _can_trigger_after_nav(self, wait_seconds: float = 10.0) -> bool:
        """
        导航启动后是否已满足等待时间（默认10秒）。
        若未启动或进程已退出，则返回 False。
        """
        if self._fastlio_proc is None or self._fastlio_proc.poll() is not None:
            return False
        if self._fastlio_started_at is None:
            return False
        return (time.time() - self._fastlio_started_at) >= wait_seconds


# -------------------------------
# 主运控模式检测器（改进版）
# -------------------------------
class G1LocoModeChecker:
    """G1机器人主运控模式检测器"""
    
    # 模式ID定义
    MODE_ZERO_TORQUE = 0      # 零力矩模式
    MODE_DAMP = 1             # 阻尼模式
    MODE_SQUAT_POS = 2        # 位控下蹲
    MODE_SIT_POS = 3          # 位控落座
    MODE_STAND_LOCK = 4       # 锁定站立
    MODE_BALANCE_SQUAT = 706  # 平衡下蹲
    MODE_STAND_UP = 500       # 常规运控（主运控）
    MODE_STAND_3DOF = 501     # 常规运控-3Dof-waist
    MODE_WALK_RUN = 801       # 走跑运控
    
    # 主运控模式列表
    MAIN_LOCO_MODES = [MODE_STAND_UP, MODE_STAND_3DOF, MODE_WALK_RUN]
    
    def __init__(self, network_interface="eth0"):
        """
        初始化检测器
        
        Args:
            network_interface: 网络接口名称
        """
        # 创建客户端
        self.loco_client = None
        
        try:
            from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
            self.loco_client = LocoClient()
            self.loco_client.SetTimeout(3.0)  # 设置超时时间
            self.loco_client.Init()
        except Exception as e:
            print(f"⚠️  初始化运控客户端失败: {e}")
    
    def GetFsmId(self):
        """
        获取当前机器人模式ID（带重试机制）
        
        Returns:
            tuple: (错误码, 模式ID)
        """
        if self.loco_client is None:
            return -1, 0
            
        try:
            # 直接调用底层API获取FSM ID
            from unitree_sdk2py.g1.loco.g1_loco_api import ROBOT_API_ID_LOCO_GET_FSM_ID
            import json
            code, data = self.loco_client._Call(ROBOT_API_ID_LOCO_GET_FSM_ID, "{}")
            
            if code == 0 and data:
                result = json.loads(data)
                mode_id = result.get("data", 0)
                return code, mode_id
            return code, 0
        except Exception as e:
            print(f"❌ 调用GetFsmId API时出错: {e}")
            return -1, 0
    
    def is_in_main_loco_mode(self):
        """
        检查机器人是否处于主运控模式
        
        Returns:
            bool: 如果处于主运控模式返回True，否则返回False
        """
        if self.loco_client is None:
            return False  # 如果无法检测，则返回False
            
        # 获取当前模式ID
        code, mode_id = self.GetFsmId()
        
        if code != 0:
            # 减少错误信息输出频率，只在必要时打印
            if not hasattr(self, '_last_error_time'):
                self._last_error_time = 0
                
            current_time = time.time()
            # 每隔5秒以上才打印一次错误信息
            if current_time - self._last_error_time > 5:
                error_messages = {
                    3102: "请求发送错误，可能是网络连接问题或服务不可用",
                    3103: "API未注册，请检查服务是否正常运行",
                    3104: "请求超时，请检查网络连接",
                    3202: "服务端内部错误",
                    3203: "API在服务端未实现",
                    3205: "请求被拒绝，可能需要更高权限"
                }
                error_desc = error_messages.get(code, "未知错误")
                print(f"⚠️  获取机器人模式失败，错误码: {code} ({error_desc})")
                self._last_error_time = current_time
            return False
            
        # 检查是否为主运控模式
        is_main_loco = mode_id in self.MAIN_LOCO_MODES
        
        # 只在模式发生变化时打印信息
        if not hasattr(self, '_last_mode_id') or self._last_mode_id != mode_id:
            self._last_mode_id = mode_id
            if is_main_loco:
                mode_names = {
                    self.MODE_STAND_UP: "常规运控（主运控）",
                    self.MODE_STAND_3DOF: "常规运控-3Dof-waist",
                    self.MODE_WALK_RUN: "走跑运控"
                }
                print(f"✅ 机器人处于主运控模式: {mode_names.get(mode_id, '未知主运控模式')}")
            else:
                mode_names = {
                    self.MODE_ZERO_TORQUE: "零力矩模式",
                    self.MODE_DAMP: "阻尼模式",
                    self.MODE_SQUAT_POS: "位控下蹲",
                    self.MODE_SIT_POS: "位控落座",
                    self.MODE_STAND_LOCK: "锁定站立",
                    self.MODE_BALANCE_SQUAT: "平衡下蹲"
                }
                print(f"🔒 机器人不处于主运控模式: {mode_names.get(mode_id, '其他模式')}")
            
        return is_main_loco
def main(return_remote=False):
   #  if len(sys.argv) < 2 and not return_remote:
   #      print("用法: python g1_client.py <网卡>")
   #      print("示例: python g1_client.py enp6s0")
   #      sys.exit(-1)

   #  network_interface = sys.argv[1] if len(sys.argv) > 1 else "lo"
    network_interface = "eth0"

    # 获取当前文件所在目录
    current_dir = os.path.dirname(os.path.abspath(__file__))
    action_dir = os.path.join(current_dir, "action")  # 默认动作目录
    
    # 自动识别网卡
    try:
        with open('/proc/net/dev', 'r') as f:
            content = f.read()
            if network_interface not in content:
                print(f"⚠️ 网卡 '{network_interface}' 不存在")
                lines = content.strip().split('\n')[2:]
                candidates = [line.split(':')[0].strip() for line in lines if 'lo' not in line and 'docker' not in line]
                if candidates:
                    network_interface = candidates[0]
                    print(f"✅ 自动选择网卡: {network_interface}")
                else:
                    network_interface = "lo"
                    print("❌ 未找到真实网卡，使用 'lo'")
    except Exception as e:
        print(f"⚠️ 网卡检测失败，使用 'lo': {e}")
        network_interface = "lo"

    # 初始化通信
    try:
        ChannelFactoryInitialize(0, network_interface)
        print(f"✅ 通信初始化成功: {network_interface}")
    except Exception as e:
        print(f"❌ 通信初始化失败: {e}")
        if not return_remote:
            sys.exit(-1)
        else:
            return None

    # 创建主运控模式检测器并在程序启动前检查主运控模式
    print("🔍 检查机器人是否处于主运控模式...")
    loco_checker = G1LocoModeChecker(network_interface)
    
    # 检查机器人是否处于主运控模式
    mode_check_count = 0
    is_in_main_mode = loco_checker.is_in_main_loco_mode()
    
    # 只检查最多10次就继续执行（每次间隔30秒）
    while not is_in_main_mode and mode_check_count < 10 and not return_remote:
        mode_check_count += 1
        print(f"⏳ 机器人未处于主运控模式，30秒后再次检查... (检查次数: {mode_check_count})")
        time.sleep(30)  # 固定等待30秒
        is_in_main_mode = loco_checker.is_in_main_loco_mode()
    
    if is_in_main_mode:
        print("✅ 机器人已处于主运控模式")
    else:
        print("⚠️  程序将继续执行，但某些功能可能受限")
        print("💡 可能的原因:")
        print("   • 机器人未开机或未站立")
        print("   • 网络连接问题")
        print("   • 机器人处于调试模式")
        print("   • 高层运动服务(ai_sport)未运行")
    
    try:
        player = G1ActionPlayer(action_dir)
        print("🔄 启动状态反馈订阅...")
    except Exception as e:
        print(f"❌ 初始化动作播放器失败: {e}")
        if not return_remote:
            sys.exit(-1)
        else:
            return None

    # 为保持代码兼容性，设置为None
    loco_client = None
    auto_task_executor = None
    
    # 创建遥控器解析器
    remote = RemoteController()
    
    # 如果只需要返回remote实例，则在此处返回
    if return_remote:
        return remote
    
    # -------------------------------
    # ROS 服务与话题（若ROS可用则启用）
    # -------------------------------
    ros_available = False
    current_dance_direction = {'value': 'A'}
    if rospy is not None and not return_remote:
        try:
            if not rospy.core.is_initialized():
                # 使用唯一节点名，允许与其他节点共存
                rospy.init_node("g1_dance_service", anonymous=True, disable_signals=True)
            ros_available = True
            print("✅ ROS 节点已初始化 (g1_dance_service)")

            def _direction_cb(msg: RosString):
                try:
                    val = msg.data
                    if val in ['Up','Down','Left','Right','A','B','X','Y']:
                        current_dance_direction['value'] = val
                        rospy.loginfo(f"dance_direction 设置为: {val}")
                    else:
                        rospy.logwarn(f"无效 dance_direction: {val}")
                except Exception as e:
                    rospy.logerr(f"direction 回调错误: {e}")

            def _handle_play_dance(_req):
                resp = TriggerResponse()
                try:
                    direction = current_dance_direction['value']
                    rospy.loginfo(f"收到 play_dance 请求: {direction}")

                    if direction not in player.actions:
                        available = list(player.actions.keys())
                        resp.success = False
                        resp.message = f"Dance '{direction}' 不存在，可用: {available}"
                        return resp

                    # 若正在播放，先停止
                    if player.state != "stopped":
                        player.stop_play()
                        wait_t0 = time.time()
                        while player.state != "stopped" and (time.time()-wait_t0) < 5.0:
                            time.sleep(0.1)

                    ok = player.play_action(direction, speed=1.0)
                    if not ok:
                        resp.success = False
                        resp.message = f"启动失败: {direction}"
                        return resp

                    # 等待完成（最多120秒）
                    t0 = time.time()
                    while player.state != "stopped" and (time.time()-t0) < 120.0:
                        time.sleep(0.1)

                    if player.state == "stopped":
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

            rospy.Subscriber("dance_direction", RosString, _direction_cb, queue_size=10)
            rospy.Service("play_dance", Trigger, _handle_play_dance)
            print("✅ ROS 服务已提供: play_dance，订阅: dance_direction")
        except Exception as e:
            print(f"⚠️ ROS 初始化失败（忽略）：{e}")

    # 创建语音识别订阅者
    try:
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
        audio_subscriber = ChannelSubscriber("rt/audio_msg", String_)
        
        # 初始化语音识别订阅
        def create_audio_handler(player_instance):
            def handler(msg: String_):
                # 只有在语音控制启用时才处理音频指令
                if player_instance.voice_control_enabled:
                    player_instance.audio_processor.process_audio_message(msg, player_instance.handle_audio_command)
                else:
                    print("🔇 语音控制已禁用，忽略语音指令")
            return handler
        
        audio_subscriber.Init(create_audio_handler(player), 10)
        player.audio_subscriber = audio_subscriber
    except Exception as e:
        print(f"⚠️  初始化语音识别订阅失败: {e}")
        player.voice_control_enabled = False  # 禁用语音控制
    
    # 等待获取初始位置反馈的标志
    state_flags = {'initial_pose_received': False, 'initialization_done': False}

    def lowstate_callback(msg):
        try:
            # 限制回调函数的处理频率，避免过度消耗CPU
            current_time = time.time()
            if not hasattr(lowstate_callback, '_last_call_time'):
                lowstate_callback._last_call_time = 0
            
            # 限制回调处理频率为50ms一次，平衡响应速度和CPU使用率
            if current_time - lowstate_callback._last_call_time < 0.05:
                return
            
            lowstate_callback._last_call_time = current_time
            
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
            player.current_pose = q_feedback
            
            # 不再设置initial_pose，使用固定的零位姿态
            if not state_flags['initial_pose_received']:
                print("🔄 首次收到位置反馈")
                state_flags['initial_pose_received'] = True
                
                # 收到反馈后执行初始化到零位
                print("🔄 开始初始化流程...")
                player.init_to_zero_position()
                print("✅ 初始化流程完成")

            # 只在首次收到反馈时显示系统就绪一次
            if state_flags['initial_pose_received'] and not state_flags['initialization_done']:
                print("✅ 系统就绪！")
                state_flags['initialization_done'] = True
                
            # 解析遥控器数据
            remote.parse(msg.wireless_remote)
            
            # 直接处理功能激活/取消激活按键，不再检查主运控模式
            # 检测F1+Start组合键（用于激活功能）
            if remote.get_combo_once('F1', 'Start'):
                player.function_activated = True  # 只有F1+Start能激活功能
                try:
                    # 添加防重复机制
                    if not hasattr(player, '_last_activation_tts_time'):
                        player._last_activation_tts_time = 0
                    current_time = time.time()
                    if current_time - player._last_activation_tts_time > 3.0:  # 至少间隔1秒
                        player.audio_processor.audio_client.TtsMaker("功能已激活", 0)
                        player._last_activation_tts_time = current_time
                    print("✅ 功能已激活")
                except Exception as e:
                    print(f"❌ 播放提示音时出错: {e}")
            
            # 检测F1+Select组合键（用于取消激活功能）
            if remote.get_combo_once('F1', 'Select'):
                # 无论当前状态如何，都立即停止所有动作并回到初始位置
                try:
                    # 添加防重复机制
                    if not hasattr(player, '_last_deactivation_tts_time'):
                        player._last_deactivation_tts_time = 0
                    current_time = time.time()
                    if current_time - player._last_deactivation_tts_time > 3.0:  # 至少间隔1秒
                        player.audio_processor.audio_client.TtsMaker("功能已取消激活", 0)
                        player._last_deactivation_tts_time = current_time
                    print("🔒 功能已取消激活")
                except Exception as e:
                    print(f"❌ 播放提示音时出错: {e}")
                
                # 立即停止当前动作并回到初始位置
                if player.state != "stopped":
                    print("⏹️ 检测到功能取消激活，立即停止当前动作并回到初始位置")
                    player.stop_play()
                    # 等待动作完全停止
                    wait_start = time.time()
                    while player.state != "stopped" and (time.time() - wait_start) < 3.0:
                        time.sleep(0.1)
                
                player.function_activated = False
                
            # 只有在功能激活状态下才处理其他按键
            if player.function_activated:
                # 检测F1+L2组合键（用于开启/关闭语音控制）
                if remote.get_combo_once('F1', 'L2'):
                    player.voice_control_enabled = not player.voice_control_enabled
                    try:
                        # 添加防重复机制
                        if not hasattr(player, '_last_voice_control_tts_time'):
                            player._last_voice_control_tts_time = 0
                        current_time = time.time()
                        if current_time - player._last_voice_control_tts_time > 3.0:  # 至少间隔1秒
                            if player.voice_control_enabled:
                                player.audio_processor.audio_client.TtsMaker("语音控制已启用", 0)
                                player._last_voice_control_tts_time = current_time
                                print("✅ 语音控制已启用")
                            else:
                                player.audio_processor.audio_client.TtsMaker("语音控制已禁用", 0)
                                player._last_voice_control_tts_time = current_time
                                print("🔒 语音控制已禁用")
                    except Exception as e:
                        print(f"❌ 播放提示音时出错: {e}")
                
                # 检查L1+F1组合键（用于取消播放）
                if remote.get_combo_once('L1', 'F1'):
                    # 不仅在playing状态，也在ramp_in状态时需要能够停止播放
                    if player.state in ["playing", "ramp_in"]:
                        print("⏹️ 检测到 L1 + F1，取消播放并回到初始姿态")
                        # 立即播放取消提示音，添加防重复机制
                        try:
                            if not hasattr(player, '_last_cancel_tts_time'):
                                player._last_cancel_tts_time = 0
                            current_time = time.time()
                            if current_time - player._last_cancel_tts_time > 3.0:  # 至少间隔1秒
                                player.audio_processor.audio_client.TtsMaker("收到取消指令，正在回到初始位置", 0)
                                player._last_cancel_tts_time = current_time
                        except Exception as e:
                            print(f"❌ 播放提示音时出错: {e}")
                        player.stop_play()
                    # 如果当前正在回到初始姿态过程中，也可以强制停止
                    elif player.state == "move_to_initial":
                        print("⏹️ 检测到 L1 + F1，强制停止回到初始姿态过程")
                        # 立即播放取消提示音，添加防重复机制
                        try:
                            if not hasattr(player, '_last_cancel_tts_time'):
                                player._last_cancel_tts_time = 0
                            current_time = time.time()
                            if current_time - player._last_cancel_tts_time > 3.0:  # 至少间隔1秒
                                player.audio_processor.audio_client.TtsMaker("收到强制停止指令", 0)
                                player._last_cancel_tts_time = current_time
                        except Exception as e:
                            print(f"❌ 播放提示音时出错: {e}")
                        # 直接设置为停止状态并停止音频
                        player.state = "stopped"
                        player._stop_audio_playback()
                        print("✅ 已强制停止动作播放")

                # 处理方向键动作播放 - 只在按键首次按下时响应
                if remote.get_combo_once('L1', 'Up'):
                    print("🎮 检测到 L1 + Up，尝试播放向上动作")
                    player.play_action('Up', speed=1.0)  # 正常速度
                elif remote.get_combo_once('L1', 'Down'):
                    print("🎮 检测到 L1 + Down，尝试播放向下动作")
                    player.play_action('Down', speed=1.0)  # 正常速度
                elif remote.get_combo_once('L1', 'Left'):
                    print("🎮 检测到 L1 + Left，尝试播放向左动作")
                    player.play_action('Left', speed=1.0)  # 正常速度
                elif remote.get_combo_once('L1', 'Right'):
                    print("🎮 检测到 L1 + Right，尝试播放向右动作")
                    player.play_action('Right', speed=1.0)  # 正常速度
                elif remote.get_combo_once('L1', 'A'):
                    print("🎮 检测到 L1 + A，尝试播放A动作")
                    player.play_action('A', speed=1.0)  # 稍快播放
                elif remote.get_combo_once('L1', 'B'):
                    print("🎮 检测到 L1 + B，尝试播放B动作")
                    player.play_action('B', speed=1.0)  # 稍慢播放
                elif remote.get_combo_once('L1', 'X'):
                    print("🎮 检测到 L1 + X，尝试播放X动作")
                    player.play_action('X', speed=1.0)  # 快速播放
                elif remote.get_combo_once('L1', 'Y'):
                    print("🎮 检测到 L1 + Y，尝试播放Y动作")
                    player.play_action('Y', speed=1.0)  # 慢速播放
                # Start + Up: 启动 fastlio 导航
                elif remote.get_combo_once('Start', 'Up'):
                    print("🚀 检测到 Start + Up，启动 fastlio 导航 (use_rviz:=false)")
                    try:
                        player._start_fastlio_navigation()
                        # 反馈一次TTS（可选）
                        if hasattr(player, 'audio_processor') and hasattr(player.audio_processor, 'audio_client'):
                            player.audio_processor.audio_client.TtsMaker("启动导航", 0)
                    except Exception as e:
                        print(f"[fastlio] 触发失败: {e}")
                # Start + Down: 导航启动≥10秒后触发 mock_dance_trigger.py
                elif remote.get_combo_once('Start', 'Down'):
                    if player._can_trigger_after_nav(10.0):
                        print("🎭 检测到 Start + Down，触发 mock_dance_trigger.py")
                        try:
                            # 在独立后台进程中运行，避免阻塞
                            subprocess.Popen([
                                "bash", "-lc",
                                "python3 /home/unitree/HongTu/PythonProject/point_nav/mock_dance_trigger.py --dance A --delay 0"
                            ])
                            if hasattr(player, 'audio_processor') and hasattr(player.audio_processor, 'audio_client'):
                                player.audio_processor.audio_client.TtsMaker("开始表演", 0)
                        except Exception as e:
                            print(f"[mock_dance] 启动失败: {e}")
                    else:
                        print("[mock_dance] 导航未满10秒，忽略 Start+Down 触发")
                
                # 处理Start+A/B/X/Y组合键，用于播放预设的TTS文本和对应动作
                elif remote.get_combo_once('Start', 'A'):
                    print("🔊 检测到 Start + A，播放预设TTS文本A和start_a目录下的动作")
                    try:
                        # 添加防重复机制
                        if not hasattr(player, '_last_tts_a_time'):
                            player._last_tts_a_time = 0
                        current_time = time.time()
                        if current_time - player._last_tts_a_time > 1.0:  # 至少间隔1秒
                            # 播放预设文本A和动作
                            player._play_tts_with_action(player.tts_presets['A'], "start_a", 0)
                            player._last_tts_a_time = current_time
                            print("✅ TTS文本A和动作播放完成")
                    except Exception as e:
                        print(f"❌ 播放TTS文本A和动作时出错: {e}")
                        
                elif remote.get_combo_once('Start', 'B'):
                    print("🔊 检测到 Start + B，播放预设TTS文本B和start_b目录下的动作")
                    try:
                        # 添加防重复机制
                        if not hasattr(player, '_last_tts_b_time'):
                            player._last_tts_b_time = 0
                        current_time = time.time()
                        if current_time - player._last_tts_b_time > 1.0:  # 至少间隔1秒
                            # 播放预设文本B和动作
                            player._play_tts_with_action(player.tts_presets['B'], "start_b", 0)
                            player._last_tts_b_time = current_time
                            print("✅ TTS文本B和动作播放完成")
                    except Exception as e:
                        print(f"❌ 播放TTS文本B和动作时出错: {e}")
                        
                elif remote.get_combo_once('Start', 'X'):
                    print("🔊 检测到 Start + X，播放预设TTS文本C和start_x目录下的动作")
                    try:
                        # 添加防重复机制
                        if not hasattr(player, '_last_tts_c_time'):
                            player._last_tts_c_time = 0
                        current_time = time.time()
                        if current_time - player._last_tts_c_time > 1.0:  # 至少间隔1秒
                            # 播放预设文本C和动作
                            player._play_tts_with_action(player.tts_presets['C'], "start_x", 0)
                            player._last_tts_c_time = current_time
                            print("✅ TTS文本C和动作播放完成")
                    except Exception as e:
                        print(f"❌ 播放TTS文本C和动作时出错: {e}")
                        
                elif remote.get_combo_once('Start', 'Y'):
                    print("🔊 检测到 Start + Y，播放预设TTS文本D和start_y目录下的动作")
                    try:
                        # 添加防重复机制
                        if not hasattr(player, '_last_tts_d_time'):
                            player._last_tts_d_time = 0
                        current_time = time.time()
                        if current_time - player._last_tts_d_time > 1.0:  # 至少间隔1秒
                            # 播放预设文本D和动作
                            player._play_tts_with_action(player.tts_presets['D'], "start_y", 0)
                            player._last_tts_d_time = current_time
                            print("✅ TTS文本D和动作播放完成")
                    except Exception as e:
                        print(f"❌ 播放TTS文本D和动作时出错: {e}")
                else:
                    # 在功能激活状态下持续更新player状态，但只在播放动作时才发送控制指令
                    player.update()
                    # 添加小延迟以降低CPU使用率
                    time.sleep(0.05)  # 50ms延迟
            else:
                # 功能未激活时，让机器人可以正常响应遥控器控制
                # 不再主动发送停止命令或回到零位命令
                # 只有在动作播放时才停止
                if player.state not in ["stopped", "move_to_initial"]:
                    print("⚠️  功能未激活，正在停止当前动作...")
                    player.stop_play()
                
                # 使用低频更新以降低CPU使用率
                player.update_low_frequency()
                time.sleep(0.1)  # 100ms延迟
        except Exception as e:
            print(f"❌ 回调处理失败: {e}")
            pass  # 添加pass语句确保语法正确
    subscriber = ChannelSubscriber("rt/lowstate", LowState_)
    subscriber.Init(lowstate_callback, 10)
    
    # 等待接收初始关节位置反馈
    print("⏳ 等待接收初始关节位置反馈...")
    wait_start = time.time()
    # 等待最多5秒，与init_to_zero_position中的一致
    while not state_flags['initial_pose_received'] and (time.time() - wait_start) < 5.0:
        time.sleep(0.1)
    
    # 即使没有收到反馈也继续执行，使用默认零位姿态
    if not state_flags['initial_pose_received']:
        print("⚠️ 超时未收到初始位置反馈，使用默认零位姿态继续执行")
        if player.current_pose is None:
            player.current_pose = np.zeros(15, dtype=np.float32)
        state_flags['initial_pose_received'] = True
        state_flags['initialization_done'] = True  # 标记为已完成初始化
    else:
        print("✅ 成功接收到初始位置反馈")
    
    # 确保player已经接收到了当前位置反馈
    if player.current_pose is None:
        player.current_pose = np.zeros(15, dtype=np.float32)
        print("🔧 使用默认零位姿态")
    
    # 程序启动后立即执行初始化到零位
    print("🔄 开始初始化流程...")
    player.init_to_zero_position()
    print("✅ 初始化流程完成")
    
    print("✅ 程序初始化完成")
    print("🔒 功能当前未激活，请在机器人处于主运控模式时按 F1 + Start 激活功能")
    print("ℹ️  程序将持续检测机器人模式状态")
    print("🎮 操作说明:")
    print("  ┌──────────────┬────────────────────────────┐")
    print("  │   按键组合   │        功能说明          │")
    print("  ├──────────────┼────────────────────────────┤")
    print("  │  L1 + Up     │  播放向上动作             │")
    print("  │  L1 + Down   │  播放向下动作             │")
    print("  │  L1 + Left   │  播放向左动作             │")
    print("  │  L1 + Right  │  播放向右动作             │")
    print("  │  L1 + A      │  播放A动作                │")
    print("  │  L1 + B      │  播放B动作                │")
    print("  │  L1 + X      │  播放X动作                │")
    print("  │  L1 + Y      │  播放Y动作                │")
    print("  │  L1 + F1     │  取消播放并回到初始姿态    │")
    print("  │  F1 + Start  │  激活功能                 │")
    print("  │  F1 + Select │  取消激活功能             │")
    print("  │  F1 + L2     │  开启/关闭语音控制        │")
    print("  │  Start + Up  │  启动 fastlio 导航        │")
    print("  │  Start + Down│  触发 mock_dance_trigger   │")
    print("  └──────────────┴────────────────────────────┘")
    print("🗣️  语音指令:")
    print("  ┌──────────────┬────────────────────────────┐")
    print("  │   指令内容   │        功能说明          │")
    print("  ├──────────────┼────────────────────────────┤")
    print("  │ 小G / 你好   │  唤醒机器人              │")
    print("  │ 播放 / 开始  │  开始播放动作             │")
    print("  │ 停止 / 结束  │  停止播放动作             │")
    print("  │    循环      │  切换循环播放模式         │")
    print("  └──────────────┴────────────────────────────┘")
    print("ℹ️  语音控制当前状态: 禁用" if not player.voice_control_enabled else "ℹ️  语音控制当前状态: 启用")
    
    # 显示加载的动作
    print("💃 已加载的动作:")
    for direction, action in player.actions.items():
        print(f"  {direction}: {action['name']}")

    try:
        while True:
            time.sleep(0.1)  # 减少CPU占用
    except KeyboardInterrupt:
        print("\n👋 收到中断信号，准备退出")
    except Exception as e:
        print(f"\n❌ 程序运行出错: {e}")
        import traceback
        traceback.print_exc()
        
    # 程序退出前确保关闭所有资源
    try:
        if player and hasattr(player, 'audio_processor'):
            # 添加防重复机制
            if not hasattr(player, '_last_exit_tts_time'):
                player._last_exit_tts_time = 0
            current_time = time.time()
            if current_time - player._last_exit_tts_time > 3.0:  # 至少间隔1秒
                player.audio_processor.audio_client.TtsMaker("程序即将退出", 0)
                player._last_exit_tts_time = current_time
    except Exception as e:
        print(f"❌ 退出提示失败: {e}")

    print("\n👋 程序退出")
    
    # 返回remote实例供其他程序使用
    return remote


if __name__ == "__main__":
    main()