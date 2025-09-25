#!/usr/bin/env python3
"""
G1机器人音频处理模块
提供语音识别、文本处理、命令解析等功能
"""

import json
import sys
import os
import re
from difflib import SequenceMatcher
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

# 添加路径以支持直接运行脚本
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

class G1AudioProcessor:
    """G1机器人音频处理器"""
    
    def __init__(self, audio_client: AudioClient = None):
        """
        初始化音频处理器
        
        Args:
            audio_client: 音频客户端实例，如果为None则自动创建
        """
        self.audio_client = audio_client or AudioClient()
        if not audio_client:
            self.audio_client.SetTimeout(10.0)
            self.audio_client.Init()
            
        # 设置音量为最大
        self.audio_client.SetVolume(100)
        
        # 定义关键词
        self.WAKE_WORDS = ["小G", "你好", "在吗", "嗨", "小羲", "金羲"]
        self.PLAY_WORDS = ["播放", "开始", "动起来", "表演"]
        self.STOP_WORDS = ["停止", "结束", "暂停", "停"]
        self.LOOP_WORDS = ["循环"]
        
        # 置信度阈值
        self.CONFIDENCE_THRESHOLD = 0.2
        
        print("🔊 音频处理器初始化完成，音量已设置为最大")
    
    def process_audio_message(self, msg: String_, player_callback=None):
        """
        处理音频消息
        
        Args:
            msg: 音频消息对象
            player_callback: 回调函数，用于执行玩家控制命令
        """
        try:
            # 解析消息数据
            data = self._parse_message_data(msg)
            if not data:
                return
                
            # 处理播放状态消息
            if "play_state" in data:
                self._handle_play_state(data)
                return
                
            # 处理ASR消息
            self._handle_asr_message(data, player_callback)
                
        except json.JSONDecodeError as e:
            print(f"❌ 语音消息JSON解析失败: {e}")
        except Exception as e:
            print(f"❌ 语音消息处理失败: {e}")
            import traceback
            traceback.print_exc()
    
    def _parse_message_data(self, msg):
        """解析消息数据"""
        # 检查msg是否是String_对象还是已经是字符串
        if hasattr(msg, 'data'):
            # 检查data是属性还是方法
            if callable(msg.data):
                raw_data = msg.data()
            else:
                raw_data = msg.data
        else:
            raw_data = str(msg)
            
        # 尝试解析JSON
        try:
            return json.loads(raw_data)
        except Exception:
            # 尝试从字符串中提取JSON对象
            start = raw_data.find("{")
            end = raw_data.rfind("}")
            if start != -1 and end != -1 and end > start:
                return json.loads(raw_data[start:end+1])
            else:
                print("[WARN] 无法从消息中提取JSON数据")
                return None
    
    def _handle_play_state(self, data):
        """处理播放状态消息"""
        play_state = data.get("play_state", 0)
        state_text = "开始播放" if play_state == 1 else "停止播放"
        print(f"🎵 音频播放状态: {state_text}")
    
    def _handle_asr_message(self, data, player_callback):
        """处理ASR消息"""
        # 提取消息信息
        text = data.get("text", "") or ""
        confidence = float(data.get("confidence", 0.0))
        is_final = data.get("is_final", True)
        angle = data.get("angle", 0)
        speaker_id = data.get("speaker_id", 0)
        sense = data.get("sense", "unknown")
        language = data.get("language", "zh-CN")
        index = data.get("index", 0)
        timestamp = data.get("timestamp", 0)
        
        # 输出识别结果
        print(f"🗣️ 语音识别 #{index}: '{text}' (置信度: {confidence:.2f}, 角度: {angle}°, 说话人: {speaker_id}, 情绪: {sense}, 语言: {language})")
        
        # 预处理文本
        processed_text = self._preprocess_text(text)
        print(f"[PREPROCESSED] 原始文本: '{text}' -> 处理后文本: '{processed_text}'")
        
        # 检查关键词并执行相应操作
        if confidence > self.CONFIDENCE_THRESHOLD:
            # 检查命令类型
            command_type = self._detect_command_type(processed_text)
            
            # 执行命令
            if command_type and player_callback:
                if command_type == 'play_named':
                    action_name = self._extract_action_name(processed_text)
                    player_callback(command_type, processed_text, action_name)
                else:
                    player_callback(command_type, processed_text)
    
    def _preprocess_text(self, text: str) -> str:
        """
        预处理识别文本，去除常见的误识别字符和噪音
        
        Args:
            text: 原始文本
            
        Returns:
            处理后的文本
        """
        if not text:
            return ""
            
        # 去除常见的误识别字符
        noise_chars = [",", ".", "，", "。", "！", "？", "?", "!", ":", "：", " ", "　"]
        for char in noise_chars:
            text = text.replace(char, "")
        
        # 处理常见的同音字错误（保留一些可能用于动作名称的字符）
        replacements = {
            "新": "羲",
            "西": "羲",
            "溪": "羲",
            "席": "羲",
            "徐": "羲",
            "机": "机",  # 保留"机"字，用于"京剧"
            "奚": "羲",
            "惜": "羲",
            "G": "G",
        }
        
        for old, new in replacements.items():
            text = text.replace(old, new)
            
        return text.strip()
    
    def _detect_command_type(self, text: str):
        """
        检测命令类型
        
        Args:
            text: 处理后的文本
            
        Returns:
            命令类型字符串
        """
        # 转换为小写以便匹配
        text_lower = text.lower()
        
        # 检查唤醒词
        for pattern in self.WAKE_WORDS:
            if re.search(pattern, text_lower):
                return 'wake'
        
        # 检查播放命令
        play_patterns = [r'播放', r'开始', r'表演', r'展示']
        for pattern in play_patterns:
            if re.search(pattern, text_lower):
                # 检查是否是指定名称的播放命令
                if len(text_lower) > len(pattern):
                    return 'play_named'
                else:
                    return 'play'
        
        # 检查停止命令
        stop_patterns = [r'停止', r'结束', r'取消', r'暂停']
        for pattern in stop_patterns:
            if re.search(pattern, text_lower):
                return 'stop'
                
        # 检查循环命令
        loop_patterns = [r'循环', r'重复']
        for pattern in loop_patterns:
            if re.search(pattern, text_lower):
                return 'loop'
        
        return None
    
    def _extract_action_name(self, text: str) -> str:
        """
        从文本中提取动作名称
        
        Args:
            text: 处理后的文本
            
        Returns:
            动作名称
        """
        # 移除常见的命令词
        command_words = ['播放', '开始', '表演', '展示', '我要', '我想', '请']
        action_name = text
        
        # 移除命令词
        for word in command_words:
            action_name = action_name.replace(word, '')
        
        # 移除标点符号
        import string
        action_name = action_name.translate(str.maketrans('', '', string.punctuation))
        
        # 去除首尾空格
        action_name = action_name.strip()
        
        return action_name
    
    def _find_matching_action(self, action_name, available_actions):
        """
        查找匹配的动作
        
        Args:
            action_name: 要查找的动作名称
            available_actions: 可用动作列表
            
        Returns:
            匹配的动作信息或None
        """
        if not action_name or not available_actions:
            return None
            
        action_name_lower = action_name.lower()
        
        # 直接匹配
        for key, action in available_actions.items():
            if action['name'].lower() == action_name_lower:
                return {'key': key, 'name': action['name']}
                
        # 模糊匹配
        best_match = None
        best_similarity = 0.0
        
        for key, action in available_actions.items():
            similarity = SequenceMatcher(None, action['name'].lower(), action_name_lower).ratio()
            if similarity > best_similarity and similarity > 0.6:  # 设置相似度阈值
                best_similarity = similarity
                best_match = {'key': key, 'name': action['name']}
                
        return best_match
    
    def handle_wake_command(self):
        """处理唤醒命令"""
        try:
            self.audio_client.TtsMaker("你好，我在的", 0)
            print("🎮 语音指令: 唤醒机器人")
        except Exception as e:
            print(f"❌ 执行唤醒操作时出错: {e}")
    
    def handle_play_command(self, player_state):
        """处理播放命令"""
        try:
            if player_state in ["stopped"]:
                self.audio_client.TtsMaker("好的，开始播放动作", 0)
                print("🎮 语音指令: 开始播放动作")
                return True
            else:
                self.audio_client.TtsMaker("当前正在播放动作", 0)
                return False
        except Exception as e:
            print(f"❌ 执行播放操作时出错: {e}")
            return False
    
    def handle_stop_command(self, player_state):
        """处理停止命令"""
        try:
            if player_state in ["playing", "ramp_in"]:
                self.audio_client.TtsMaker("好的，停止播放动作", 0)
                print("🎮 语音指令: 停止播放动作")
                return True
            else:
                self.audio_client.TtsMaker("当前没有播放动作", 0)
                return False
        except Exception as e:
            print(f"❌ 执行停止操作时出错: {e}")
            return False
    
    def handle_loop_command(self, current_loop_state):
        """处理循环命令"""
        try:
            new_loop_state = not current_loop_state
            if new_loop_state:
                self.audio_client.TtsMaker("已开启循环播放", 0)
                print("🎮 语音指令: 开启循环播放")
            else:
                self.audio_client.TtsMaker("已关闭循环播放", 0)
                print("🎮 语音指令: 关闭循环播放")
            return new_loop_state
        except Exception as e:
            print(f"❌ 执行循环操作时出错: {e}")
            return current_loop_state
    
    def handle_play_named_command(self, player_state, action_name, available_actions):
        """
        处理播放指定名称动作的命令
        
        Args:
            player_state: 当前播放状态
            action_name: 要播放的动作名称
            available_actions: 可用动作列表
            
        Returns:
            是否成功处理命令
        """
        try:
            if player_state not in ["stopped"]:
                self.audio_client.TtsMaker("当前正在播放动作", 0)
                return False
                
            # 查找匹配的动作
            matched_action = self._find_matching_action(action_name, available_actions)
            
            if matched_action:
                self.audio_client.TtsMaker(f"好的，开始播放{matched_action['name']}", 0)
                print(f"🎮 语音指令: 播放指定动作 '{matched_action['name']}'")
                return matched_action['key']  # 返回动作键
            else:
                self.audio_client.TtsMaker(f"抱歉，我没有学会{action_name}", 0)
                print(f"🎮 语音指令: 尝试播放动作 '{action_name}'，但未找到匹配项")
                return None
                
        except Exception as e:
            print(f"❌ 执行播放指定动作操作时出错: {e}")
            return None
