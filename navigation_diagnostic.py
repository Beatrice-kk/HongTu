#!/usr/bin/env python3
"""
导航系统诊断脚本
用于检查路径规划失败的原因
"""

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
from costmap_2d.msg import Costmap2D
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon
import yaml
import os

class NavigationDiagnostic:
    def __init__(self):
        rospy.init_node('navigation_diagnostic')
        
        # 订阅话题
        self.map_sub = rospy.Subscriber('/map_2d', OccupancyGrid, self.map_callback)
        self.global_costmap_sub = rospy.Subscriber('/move_base_flex/global_costmap/costmap', Costmap2D, self.global_costmap_callback)
        self.local_costmap_sub = rospy.Subscriber('/move_base_flex/local_costmap/costmap', Costmap2D, self.local_costmap_callback)
        
        # 存储数据
        self.map_data = None
        self.global_costmap_data = None
        self.local_costmap_data = None
        
        # 机器人参数
        self.robot_footprint = [
            [0.15, 0.0], [0.106, 0.106], [0.0, 0.15],
            [-0.106, 0.106], [-0.15, 0.0], [-0.106, -0.106],
            [0.0, -0.15], [0.106, -0.106]
        ]
        
        rospy.loginfo("导航诊断系统已启动")
        
    def map_callback(self, msg):
        self.map_data = msg
        rospy.loginfo("收到地图数据")
        
    def global_costmap_callback(self, msg):
        self.global_costmap_data = msg
        rospy.loginfo("收到全局代价地图数据")
        
    def local_costmap_callback(self, msg):
        self.local_costmap_data = msg
        rospy.loginfo("收到局部代价地图数据")
        
    def analyze_costmap(self, costmap_data, name):
        """分析代价地图的障碍物分布"""
        if costmap_data is None:
            rospy.logwarn(f"{name} 数据未收到")
            return
            
        # 统计不同代价值的像素数量
        cost_values = {}
        for cost in costmap_data.data:
            if cost in cost_values:
                cost_values[cost] += 1
            else:
                cost_values[cost] = 1
                
        rospy.loginfo(f"{name} 代价分布:")
        for cost, count in sorted(cost_values.items()):
            if cost > 0:  # 只显示有代价的区域
                rospy.loginfo(f"  代价 {cost}: {count} 个像素")
                
        # 计算障碍物密度
        total_pixels = len(costmap_data.data)
        obstacle_pixels = sum(1 for cost in costmap_data.data if cost > 0)
        density = obstacle_pixels / total_pixels * 100
        
        rospy.loginfo(f"{name} 障碍物密度: {density:.2f}%")
        
        return density
        
    def check_robot_footprint(self):
        """检查机器人足迹是否合理"""
        rospy.loginfo("机器人足迹检查:")
        rospy.loginfo(f"  足迹顶点数: {len(self.robot_footprint)}")
        
        # 计算足迹面积
        area = 0
        for i in range(len(self.robot_footprint)):
            j = (i + 1) % len(self.robot_footprint)
            area += self.robot_footprint[i][0] * self.robot_footprint[j][1]
            area -= self.robot_footprint[j][0] * self.robot_footprint[i][1]
        area = abs(area) / 2
        
        rospy.loginfo(f"  足迹面积: {area:.4f} 平方米")
        
        # 检查足迹是否过大
        if area > 0.1:  # 0.1平方米
            rospy.logwarn("  警告: 机器人足迹可能过大，可能影响路径规划")
        else:
            rospy.loginfo("  机器人足迹大小合理")
            
    def suggest_optimizations(self):
        """根据分析结果提供优化建议"""
        rospy.loginfo("=== 优化建议 ===")
        
        if self.global_costmap_data:
            density = self.analyze_costmap(self.global_costmap_data, "全局代价地图")
            if density > 20:
                rospy.logwarn("  建议: 全局代价地图障碍物密度过高，考虑:")
                rospy.logwarn("    - 减少膨胀半径")
                rospy.logwarn("    - 调整传感器参数")
                rospy.logwarn("    - 检查地图质量")
            else:
                rospy.loginfo("  全局代价地图障碍物密度正常")
                
        if self.local_costmap_data:
            density = self.analyze_costmap(self.local_costmap_data, "局部代价地图")
            if density > 15:
                rospy.logwarn("  建议: 局部代价地图障碍物密度过高，考虑:")
                rospy.logwarn("    - 减少局部膨胀半径")
                rospy.logwarn("    - 调整传感器检测范围")
            else:
                rospy.loginfo("  局部代价地图障碍物密度正常")
                
        rospy.loginfo("  通用建议:")
        rospy.loginfo("    - 确保地图质量良好，无明显噪声")
        rospy.loginfo("    - 检查传感器标定是否正确")
        rospy.loginfo("    - 考虑使用更宽松的规划器参数")
        rospy.loginfo("    - 增加规划器超时时间")
        
    def run_diagnostic(self):
        """运行完整诊断"""
        rospy.loginfo("开始导航系统诊断...")
        
        # 等待数据
        rospy.loginfo("等待传感器数据...")
        timeout = 10  # 10秒超时
        start_time = rospy.Time.now()
        
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            if self.map_data and self.global_costmap_data:
                break
            rospy.sleep(0.1)
            
        if not self.map_data:
            rospy.logerr("未收到地图数据，请检查地图发布")
            return
            
        if not self.global_costmap_data:
            rospy.logerr("未收到全局代价地图数据，请检查导航节点")
            return
            
        # 执行分析
        rospy.loginfo("=== 诊断结果 ===")
        self.check_robot_footprint()
        self.analyze_costmap(self.global_costmap_data, "全局代价地图")
        
        if self.local_costmap_data:
            self.analyze_costmap(self.local_costmap_data, "局部代价地图")
            
        self.suggest_optimizations()
        
        rospy.loginfo("诊断完成")

def main():
    diagnostic = NavigationDiagnostic()
    
    # 等待一段时间让数据稳定
    rospy.sleep(2)
    
    # 运行诊断
    diagnostic.run_diagnostic()
    
    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    main()
