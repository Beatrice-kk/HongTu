#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import os
import argparse
from enum import Enum

# 添加当前目录到Python路径
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

# 导入配置加载器
from config_loader import DanceConfigLoader

# 导入原有的导航类（稍后重构）
sys.path.insert(0, os.path.join(os.path.dirname(current_dir), ".."))
from simplified_nav_dance import SimpleNavWaypointPlayer


class DanceNavigationNode:
    """舞蹈导航节点"""
    
    def __init__(self, dance_type: str, config_file: str = None, debug: bool = False):
        """
        初始化舞蹈导航节点
        
        Args:
            dance_type: 舞蹈类型
            config_file: 配置文件路径
            debug: 是否启用调试模式
        """
        self.dance_type = dance_type
        self.debug = debug
        
        # 加载配置
        self.config_loader = DanceConfigLoader(config_file)
        
        # 验证配置
        if not self.config_loader.validate_config():
            rospy.logerr("配置文件验证失败，程序退出")
            sys.exit(1)
        
        # 获取配置
        self._load_configuration()
        
        # 初始化导航器
        self._initialize_navigator()
    
    def _load_configuration(self):
        """加载配置参数"""
        rospy.loginfo("加载配置参数...")
        
        # 基础配置
        self.backstage_pos = self.config_loader.get_backstage_pos()
        self.threshold = self.config_loader.get_threshold()
        self.waypoint_timeout = self.config_loader.get_waypoint_timeout()
        self.max_goal_retries = self.config_loader.get_max_goal_retries()
        
        # 舞蹈编排
        self.dance_waypoints = self.config_loader.get_dance_choreography(self.dance_type)
        
        # TTS配置
        self.tts_config = self.config_loader.get_tts_config(self.dance_type)
        
        # 服务配置
        self.service_config = self.config_loader.get_service_config()
        
        # 话题配置
        self.topic_config = self.config_loader.get_topic_config()
        
        # 定时器配置
        self.timer_config = self.config_loader.get_timer_config()
        
        rospy.loginfo(f"配置加载完成 - 舞蹈类型: {self.dance_type}")
        rospy.loginfo(f"后台位置: {self.backstage_pos}")
        rospy.loginfo(f"航点数量: {len(self.dance_waypoints)}")
    
    def _initialize_navigator(self):
        """初始化导航器"""
        rospy.loginfo("初始化导航器...")
        
        # 转换舞蹈编排格式
        dance_choreography = self._convert_dance_choreography()
        
        # 创建导航器
        self.navigator = SimpleNavWaypointPlayer(
            backstage_pos=self.backstage_pos,
            dance_type=self.dance_type,
            dance_choreography=dance_choreography
        )
        
        rospy.loginfo("导航器初始化完成")
    
    def _convert_dance_choreography(self):
        """转换舞蹈编排格式以兼容原有代码"""
        choreography = {}
        
        # 转换航点格式
        waypoints = []
        wait_times = []
        
        for waypoint in self.dance_waypoints:
            position = waypoint['position']
            wait_time = waypoint['wait_time']
            waypoints.append(tuple(position))
            wait_times.append(wait_time)
        
        # 创建兼容格式
        choreography[self.dance_type] = list(zip(waypoints, wait_times))
        
        return choreography
    
    def run(self):
        """运行导航节点"""
        rospy.loginfo(f"开始舞蹈导航 - 类型: {self.dance_type}")
        
        try:
            # 启动导航
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("接收到中断信号，正在关闭...")
        except Exception as e:
            rospy.logerr(f"导航过程中发生错误: {e}")
        finally:
            rospy.loginfo("舞蹈导航节点关闭")


def main():
    """主函数"""
    # 初始化ROS节点
    rospy.init_node("dance_navigation_node")
    
    # 解析命令行参数，过滤掉ROS参数
    parser = argparse.ArgumentParser(description="舞蹈导航节点")
    parser.add_argument("--dance", type=str, default="A", 
                       choices=["A", "B", "X", "Y", "Up", "Down"],
                       help="舞蹈类型")
    parser.add_argument("--config", type=str, default=None,
                       help="配置文件路径")
    parser.add_argument("--debug", action="store_true",
                       help="启用调试模式")
    
    # 使用rospy.myargv()过滤掉ROS参数
    args = parser.parse_args(rospy.myargv()[1:])
    
    # 获取ROS参数（如果通过launch文件启动）
    dance_type = rospy.get_param("~dance_type", args.dance)
    config_file = rospy.get_param("~config_file", args.config)
    debug = rospy.get_param("~debug", args.debug)
    
    rospy.loginfo(f"启动参数 - 舞蹈类型: {dance_type}, 配置文件: {config_file}, 调试: {debug}")
    
    try:
        # 创建并运行节点
        node = DanceNavigationNode(
            dance_type=dance_type,
            config_file=config_file,
            debug=debug
        )
        node.run()
        
    except Exception as e:
        rospy.logerr(f"节点启动失败: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
