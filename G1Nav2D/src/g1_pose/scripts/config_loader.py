#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import yaml
import rospy
import os
from typing import Dict, List, Tuple, Any


class DanceConfigLoader:
    """舞蹈配置加载器"""
    
    def __init__(self, config_file: str = None):
        """
        初始化配置加载器
        
        Args:
            config_file: 配置文件路径，如果为None则使用默认路径
        """
        if config_file is None:
            # 默认配置文件路径
            package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            config_file = os.path.join(package_path, "config", "dance_config.yaml")
        
        self.config_file = config_file
        self.config = self._load_config()
    
    def _load_config(self) -> Dict[str, Any]:
        """加载配置文件"""
        try:
            with open(self.config_file, 'r', encoding='utf-8') as file:
                config = yaml.safe_load(file)
            rospy.loginfo(f"成功加载配置文件: {self.config_file}")
            return config
        except FileNotFoundError:
            rospy.logerr(f"配置文件未找到: {self.config_file}")
            raise
        except yaml.YAMLError as e:
            rospy.logerr(f"配置文件解析错误: {e}")
            raise
        except Exception as e:
            rospy.logerr(f"加载配置文件失败: {e}")
            raise
    
    def get_basic_config(self) -> Dict[str, Any]:
        """获取基础配置"""
        return self.config.get('basic_config', {})
    
    def get_dance_choreography(self, dance_type: str) -> List[Dict[str, Any]]:
        """
        获取指定舞蹈类型的编排
        
        Args:
            dance_type: 舞蹈类型 (A, B, X, Y, Up, Down)
            
        Returns:
            舞蹈编排列表，每个元素包含position和wait_time
        """
        choreography = self.config.get('dance_choreography', {})
        if dance_type not in choreography:
            rospy.logwarn(f"舞蹈类型 '{dance_type}' 未找到，使用默认编排")
            # 返回第一个可用的编排
            if choreography:
                dance_type = list(choreography.keys())[0]
            else:
                raise ValueError("没有可用的舞蹈编排")
        
        waypoints = choreography[dance_type].get('waypoints', [])
        return waypoints
    
    def get_tts_config(self, dance_type: str) -> Dict[str, str]:
        """
        获取TTS配置
        
        Args:
            dance_type: 舞蹈类型
            
        Returns:
            TTS配置字典，包含text和action_dir
        """
        tts_config = self.config.get('tts_config', {})
        if dance_type not in tts_config:
            rospy.logwarn(f"舞蹈类型 '{dance_type}' 的TTS配置未找到")
            return {}
        
        return tts_config[dance_type]
    
    def get_service_config(self) -> Dict[str, Any]:
        """获取服务配置"""
        return self.config.get('service_config', {})
    
    def get_topic_config(self) -> Dict[str, str]:
        """获取话题配置"""
        return self.config.get('topic_config', {})
    
    def get_timer_config(self) -> Dict[str, float]:
        """获取定时器配置"""
        return self.config.get('timer_config', {})
    
    def get_backstage_pos(self) -> Tuple[float, float, float]:
        """获取后台位置"""
        basic_config = self.get_basic_config()
        pos = basic_config.get('backstage_pos', [-0.6, 0, 0])
        return tuple(pos)
    
    def get_threshold(self) -> float:
        """获取到达阈值"""
        basic_config = self.get_basic_config()
        return basic_config.get('threshold', 0.5)
    
    def get_waypoint_timeout(self) -> float:
        """获取航点超时时间"""
        basic_config = self.get_basic_config()
        return basic_config.get('waypoint_timeout', 20.0)
    
    def get_max_goal_retries(self) -> int:
        """获取最大重试次数"""
        basic_config = self.get_basic_config()
        return basic_config.get('max_goal_retries', 2)
    
    def validate_config(self) -> bool:
        """验证配置文件的完整性"""
        try:
            # 检查必需的配置项
            required_sections = ['basic_config', 'dance_choreography', 'topic_config']
            for section in required_sections:
                if section not in self.config:
                    rospy.logerr(f"配置文件缺少必需部分: {section}")
                    return False
            
            # 检查舞蹈编排
            choreography = self.config.get('dance_choreography', {})
            if not choreography:
                rospy.logerr("舞蹈编排配置为空")
                return False
            
            # 检查每个舞蹈编排
            for dance_type, dance_config in choreography.items():
                if 'waypoints' not in dance_config:
                    rospy.logerr(f"舞蹈类型 '{dance_type}' 缺少waypoints配置")
                    return False
                
                waypoints = dance_config['waypoints']
                if not waypoints:
                    rospy.logerr(f"舞蹈类型 '{dance_type}' 的waypoints为空")
                    return False
                
                # 检查每个航点
                for i, waypoint in enumerate(waypoints):
                    if 'position' not in waypoint:
                        rospy.logerr(f"舞蹈类型 '{dance_type}' 的航点 {i} 缺少position")
                        return False
                    
                    position = waypoint['position']
                    if len(position) != 3:
                        rospy.logerr(f"舞蹈类型 '{dance_type}' 的航点 {i} position格式错误")
                        return False
            
            rospy.loginfo("配置文件验证通过")
            return True
            
        except Exception as e:
            rospy.logerr(f"配置文件验证失败: {e}")
            return False
    
    def reload_config(self):
        """重新加载配置文件"""
        rospy.loginfo("重新加载配置文件...")
        self.config = self._load_config()
        rospy.loginfo("配置文件重新加载完成")
