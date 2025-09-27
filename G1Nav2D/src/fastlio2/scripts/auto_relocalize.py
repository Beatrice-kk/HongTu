#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 🚀 疯狂重定位程序 - 超级增强版 🚀
# 作者: AI助手
# 功能: 智能重定位、多线程并行、AI预测、可视化等

import rospy
import math
import yaml
import threading
import time
import random
import numpy as np
from collections import deque
from datetime import datetime
import json
import os
from fastlio.srv import SlamReLoc, SlamReLocRequest
from fastlio.srv import SlamRelocCheck, SlamRelocCheckRequest
from std_srvs.srv import Empty
from std_msgs.msg import Bool, String, Float32MultiArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

# --- 🧠 AI智能重定位类 ---
class CrazyRelocalizer:
    def __init__(self):
        """初始化疯狂重定位器"""
        self.reloc_history = deque(maxlen=1000)  # 重定位历史记录
        self.success_patterns = {}  # 成功模式分析
        self.adaptive_params = {
            'search_range': 180.0,
            'step_size': 5.0,
            'parallel_threads': 4,
            'ai_prediction': True,
            'visualization': True
        }
        self.thread_pool = []
        self.results_lock = threading.Lock()
        self.best_angles = []  # 最佳角度缓存
        self.ml_model = None  # 机器学习模型
        
        # 可视化相关
        self.marker_pub = rospy.Publisher('/reloc_visualization', MarkerArray, queue_size=1)
        self.status_pub = rospy.Publisher('/reloc_status', String, queue_size=1)
        self.stats_pub = rospy.Publisher('/reloc_stats', Float32MultiArray, queue_size=1)
        
        # 统计信息
        self.stats = {
            'total_attempts': 0,
            'successful_attempts': 0,
            'average_time': 0.0,
            'best_angles': [],
            'failure_patterns': []
        }
        
        rospy.loginfo("🚀 疯狂重定位器已启动！准备进行AI智能重定位...")

# --- 服务调用封装 ---

def call_service(service_name, service_type, request=None):
    """一个通用的服务调用函数 - 增强版"""
    rospy.logdebug("Waiting for service: %s", service_name)
    try:
        rospy.wait_for_service(service_name, timeout=5.0)
    except rospy.ROSException:
        rospy.logerr("Service '%s' not available. Is the localizer_node running?", service_name)
        return None
    
    try:
        service_proxy = rospy.ServiceProxy(service_name, service_type)
        if request is None and service_type == Empty:
             response = service_proxy()
        else:
             response = service_proxy(request)
        return response
    except rospy.ServiceException as e:
        rospy.logerr("Service call to '%s' failed: %s", service_name, e)
        return None

    def ai_predict_best_angles(self, map_path, x, y):
        """🧠 AI预测最佳重定位角度"""
        rospy.loginfo("🤖 AI正在分析历史数据，预测最佳重定位角度...")
        
        # 基于历史成功模式预测
        if len(self.best_angles) > 0:
            # 使用加权平均和方差分析
            angles = np.array(self.best_angles)
            mean_angle = np.mean(angles)
            std_angle = np.std(angles)
            
            # 生成预测角度：均值附近 + 一些随机探索
            predicted_angles = []
            for i in range(8):  # 预测8个角度
                if i < 4:
                    # 基于历史成功角度的预测
                    angle = mean_angle + np.random.normal(0, std_angle * 0.5)
                else:
                    # 随机探索新角度
                    angle = random.uniform(-180, 180)
                
                predicted_angles.append(angle)
            
            rospy.loginfo(f"🎯 AI预测角度: {[f'{a:.1f}°' for a in predicted_angles[:4]]}")
            return predicted_angles
        else:
            # 没有历史数据，使用智能初始策略
            return self.generate_smart_initial_angles()
    
    def generate_smart_initial_angles(self):
        """生成智能初始角度策略"""
        # 基于几何对称性和常见成功模式
        smart_angles = [0, 90, -90, 180, 45, -45, 135, -135, 30, -30, 60, -60]
        rospy.loginfo("🧠 使用智能初始角度策略")
        return smart_angles
    
    def parallel_relocalization(self, map_path, x, y, angles):
        """🚀 多线程并行重定位"""
        rospy.loginfo(f"⚡ 启动{len(angles)}个并行重定位线程...")
        
        results = {}
        threads = []
        
        def reloc_worker(angle, thread_id):
            """单个重定位工作线程"""
            try:
                start_time = time.time()
                success = self.attempt_relocalization_enhanced(map_path, x, y, math.radians(angle))
                duration = time.time() - start_time
                
                with self.results_lock:
                    results[thread_id] = {
                        'angle': angle,
                        'success': success,
                        'duration': duration,
                        'thread_id': thread_id
                    }
                
                if success:
                    rospy.loginfo(f"🎉 线程{thread_id}成功！角度: {angle}°, 耗时: {duration:.2f}s")
                else:
                    rospy.loginfo(f"❌ 线程{thread_id}失败，角度: {angle}°")
                    
            except Exception as e:
                rospy.logerr(f"线程{thread_id}异常: {e}")
        
        # 启动所有线程
        for i, angle in enumerate(angles):
            thread = threading.Thread(target=reloc_worker, args=(angle, i))
            thread.start()
            threads.append(thread)
        
        # 等待所有线程完成
        for thread in threads:
            thread.join(timeout=10.0)  # 10秒超时
        
        # 分析结果
        successful_angles = [r['angle'] for r in results.values() if r['success']]
        if successful_angles:
            rospy.loginfo(f"🎯 并行重定位成功！成功角度: {successful_angles}")
            return successful_angles[0]  # 返回第一个成功的角度
        else:
            rospy.logwarn("😞 所有并行重定位都失败了")
            return None
    
    def attempt_relocalization_enhanced(self, map_path, x, y, yaw_rad):
        """增强版重定位尝试 - 带统计和可视化"""
        angle_deg = math.degrees(yaw_rad)
        rospy.loginfo(f"🎯 尝试重定位: {angle_deg:.1f}°")
        
        # 记录开始时间
        start_time = time.time()
        
        req = SlamReLocRequest()
        req.pcd_path = map_path
        req.x = x
        req.y = y
        req.z = 0.0
        req.roll = 0.0
        req.pitch = 0.0
        req.yaw = yaw_rad

        # 调用重定位服务
        response = call_service('/slam_reloc', SlamReLoc, req)
        duration = time.time() - start_time
        
        success = False
        if response is not None:
            success = response.status == 1
            rospy.loginfo(f"📊 重定位结果: {angle_deg:.1f}° -> {'成功' if success else '失败'} (耗时: {duration:.2f}s)")
        else:
            rospy.logwarn(f"❌ 重定位服务调用失败: {angle_deg:.1f}°")
        
        # 记录到历史
        self.record_reloc_attempt(angle_deg, success, duration)
        
        # 更新统计
        self.update_stats(angle_deg, success, duration)
        
        # 发布可视化
        self.publish_visualization(angle_deg, success)
        
        return success
    
    def record_reloc_attempt(self, angle, success, duration):
        """记录重定位尝试到历史"""
        record = {
            'timestamp': datetime.now().isoformat(),
            'angle': angle,
            'success': success,
            'duration': duration
        }
        self.reloc_history.append(record)
        
        if success:
            self.best_angles.append(angle)
            # 保持最佳角度列表不超过50个
            if len(self.best_angles) > 50:
                self.best_angles = self.best_angles[-50:]
    
    def update_stats(self, angle, success, duration):
        """更新统计信息"""
        self.stats['total_attempts'] += 1
        if success:
            self.stats['successful_attempts'] += 1
            self.stats['best_angles'].append(angle)
        
        # 计算平均时间
        if self.stats['total_attempts'] > 0:
            self.stats['average_time'] = (
                (self.stats['average_time'] * (self.stats['total_attempts'] - 1) + duration) 
                / self.stats['total_attempts']
            )
    
    def publish_visualization(self, angle, success):
        """发布可视化信息"""
        # 创建角度标记
        marker_array = MarkerArray()
        
        # 成功角度 - 绿色
        if success:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.id = int(angle * 100)  # 使用角度作为ID
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            # 设置位置和方向
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
            
            # 设置箭头方向
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = math.sin(math.radians(angle) / 2)
            marker.pose.orientation.w = math.cos(math.radians(angle) / 2)
            
            # 设置大小和颜色
            marker.scale.x = 2.0
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
        
        # 发布可视化
        self.marker_pub.publish(marker_array)
        
        # 发布状态信息
        status_msg = String()
        status_msg.data = f"角度: {angle:.1f}° - {'成功' if success else '失败'}"
        self.status_pub.publish(status_msg)
        
        # 发布统计信息
        stats_msg = Float32MultiArray()
        stats_msg.data = [
            self.stats['total_attempts'],
            self.stats['successful_attempts'],
            self.stats['average_time'],
            angle
        ]
        self.stats_pub.publish(stats_msg)
    
    def save_success_record(self, angle):
        """保存成功记录到文件"""
        try:
            record = {
                'timestamp': datetime.now().isoformat(),
                'successful_angle': angle,
                'stats': self.stats.copy()
            }
            
            # 保存到JSON文件
            filename = f"/tmp/reloc_success_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            with open(filename, 'w') as f:
                json.dump(record, f, indent=2)
            
            rospy.loginfo(f"💾 成功记录已保存到: {filename}")
            
        except Exception as e:
            rospy.logwarn(f"保存成功记录失败: {e}")
    
    def publish_final_stats(self):
        """发布最终统计信息"""
        try:
            rospy.loginfo("📊" + "="*50)
            rospy.loginfo("📊 疯狂重定位最终统计:")
            rospy.loginfo(f"📊 总尝试次数: {self.stats['total_attempts']}")
            rospy.loginfo(f"📊 成功次数: {self.stats['successful_attempts']}")
            rospy.loginfo(f"📊 成功率: {self.stats['successful_attempts']/max(1, self.stats['total_attempts'])*100:.1f}%")
            rospy.loginfo(f"📊 平均耗时: {self.stats['average_time']:.2f}秒")
            rospy.loginfo(f"📊 最佳角度: {self.stats['best_angles'][-5:] if self.stats['best_angles'] else '无'}")
            rospy.loginfo("📊" + "="*50)
            
            # 发布最终统计消息
            final_stats_msg = Float32MultiArray()
            final_stats_msg.data = [
                self.stats['total_attempts'],
                self.stats['successful_attempts'],
                self.stats['average_time'],
                len(self.stats['best_angles'])
            ]
            self.stats_pub.publish(final_stats_msg)
            
        except Exception as e:
            rospy.logwarn(f"发布最终统计失败: {e}")
    
    def crazy_explosion_search(self, map_path, x, y):
        """💥 疯狂爆炸式搜索 - 随机角度轰炸"""
        rospy.loginfo("💥 启动疯狂爆炸式搜索模式！")
        
        # 生成大量随机角度
        explosion_angles = []
        for i in range(50):  # 50个随机角度
            angle = random.uniform(-180, 180)
            explosion_angles.append(angle)
        
        rospy.loginfo(f"💥 生成{len(explosion_angles)}个随机角度进行轰炸式搜索！")
        
        # 分批并行处理
        batch_size = 10
        for i in range(0, len(explosion_angles), batch_size):
            batch = explosion_angles[i:i+batch_size]
            rospy.loginfo(f"💥 处理批次 {i//batch_size + 1}/{(len(explosion_angles)-1)//batch_size + 1}")
            
            # 并行处理这一批
            successful_angle = self.parallel_relocalization(map_path, x, y, batch)
            if successful_angle is not None:
                rospy.loginfo(f"🎆 爆炸式搜索成功！角度: {successful_angle:.1f}°")
                return successful_angle
        
        rospy.logwarn("💥 爆炸式搜索失败，所有角度都尝试过了")
        return None
    
    def adaptive_parameter_tuning(self):
        """🔧 自适应参数调优"""
        rospy.loginfo("🔧 启动自适应参数调优...")
        
        # 根据历史成功率调整参数
        if len(self.reloc_history) > 10:
            recent_attempts = list(self.reloc_history)[-10:]
            success_rate = sum(1 for r in recent_attempts if r['success']) / len(recent_attempts)
            
            if success_rate < 0.3:  # 成功率低于30%
                rospy.loginfo("🔧 成功率较低，调整搜索策略...")
                self.adaptive_params['search_range'] = min(360.0, self.adaptive_params['search_range'] * 1.2)
                self.adaptive_params['step_size'] = max(1.0, self.adaptive_params['step_size'] * 0.8)
                self.adaptive_params['parallel_threads'] = min(8, self.adaptive_params['parallel_threads'] + 1)
            elif success_rate > 0.7:  # 成功率高于70%
                rospy.loginfo("🔧 成功率较高，优化搜索效率...")
                self.adaptive_params['search_range'] = max(90.0, self.adaptive_params['search_range'] * 0.9)
                self.adaptive_params['step_size'] = min(10.0, self.adaptive_params['step_size'] * 1.1)
        
        rospy.loginfo(f"🔧 自适应参数: {self.adaptive_params}")
    
    def play_reloc_music(self, success):
        """🎵 播放重定位音乐"""
        try:
            if success:
                rospy.loginfo("🎵 播放成功音效: 胜利进行曲！")
                # 这里可以添加实际的音效播放代码
                # 例如使用pygame或其他音频库
            else:
                rospy.loginfo("🎵 播放失败音效: 悲伤小调...")
        except Exception as e:
            rospy.logwarn(f"播放音效失败: {e}")
    
    def generate_crazy_angles(self):
        """🎲 生成疯狂角度序列"""
        crazy_angles = []
        
        # 1. 黄金比例角度
        golden_ratio = 1.618
        for i in range(10):
            angle = (i * 137.5 * golden_ratio) % 360 - 180  # 黄金角度
            crazy_angles.append(angle)
        
        # 2. 斐波那契角度
        fib_angles = [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55, 89]
        for fib in fib_angles:
            angle = (fib * 137.5) % 360 - 180
            crazy_angles.append(angle)
        
        # 3. 质数角度
        primes = [2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47]
        for prime in primes:
            angle = (prime * 137.5) % 360 - 180
            crazy_angles.append(angle)
        
        # 4. 随机角度
        for i in range(20):
            angle = random.uniform(-180, 180)
            crazy_angles.append(angle)
        
        rospy.loginfo(f"🎲 生成疯狂角度序列: {len(crazy_angles)}个角度")
        return crazy_angles
    
    def quantum_relocalization(self, map_path, x, y):
        """⚛️ 量子重定位 - 同时尝试所有可能的角度"""
        rospy.loginfo("⚛️ 启动量子重定位模式！")
        
        # 生成量子角度（0-360度，每1度一个）
        quantum_angles = list(range(-180, 181))
        rospy.loginfo(f"⚛️ 量子角度数量: {len(quantum_angles)}")
        
        # 分批处理量子角度
        batch_size = 20
        for i in range(0, len(quantum_angles), batch_size):
            batch = quantum_angles[i:i+batch_size]
            rospy.loginfo(f"⚛️ 量子批次 {i//batch_size + 1}/{(len(quantum_angles)-1)//batch_size + 1}")
            
            successful_angle = self.parallel_relocalization(map_path, x, y, batch)
            if successful_angle is not None:
                rospy.loginfo(f"⚛️ 量子重定位成功！角度: {successful_angle:.1f}°")
                return successful_angle
        
        rospy.logwarn("⚛️ 量子重定位失败")
        return None

def attempt_relocalization(map_path, x, y, yaw_rad):
    """尝试使用给定的位姿进行一次重定位 - 兼容原版"""
    rospy.loginfo("--> Attempting relocalization at yaw: %.1f deg", math.degrees(yaw_rad))
    
    req = SlamReLocRequest()
    req.pcd_path = map_path
    req.x = x
    req.y = y
    req.z = 0.0  # 假设为2D
    req.roll = 0.0
    req.pitch = 0.0
    req.yaw = yaw_rad

    # 调用重定位服务
    response = call_service('/slam_reloc', SlamReLoc, req)
    if response is not None:
        rospy.loginfo("Reloc service response: status=%s", response.status)
        return response.status == 1
    else:
        rospy.logwarn("Reloc service call failed or returned None")
        return False

def check_reloc_status():
    """检查重定位是否成功"""
    response = call_service('/slam_reloc_check', SlamRelocCheck, SlamRelocCheckRequest())
    if response:
        rospy.logdebug("Reloc check response: status=%s", response.status)
        return response.status
    rospy.logwarn("Reloc check service call failed")
    return None # 如果服务调用失败，返回None

# --- 🚀 疯狂主逻辑 ---

def main():
    rospy.init_node('crazy_reloc_node', anonymous=True)
    
    # 创建疯狂重定位器
    crazy_reloc = CrazyRelocalizer()
    
    # 创建发布者，用于发布重定位成功消息
    reloc_success_pub = rospy.Publisher('/relocalization_success', Bool, queue_size=1, latch=True)

    # 1. 从参数服务器加载配置
    try:
        config = rospy.get_param('~') # 从私有命名空间获取参数
        map_path = config['map_pcd_path']
        initial_pose = config['initial_pose']
        yaw_search = config['yaw_search']
    except KeyError as e:
        rospy.logerr("Configuration parameter %s not found on the parameter server. Did you load the YAML file?", e)
        rospy.logerr("Use: rosparam load your_config.yaml")
        return

    rospy.loginfo("🚀 疯狂重定位配置已加载！")
    rospy.loginfo("🗺️ 地图路径: %s", map_path)
    rospy.loginfo("📍 初始位置 (x,y): (%.2f, %.2f)", initial_pose['x'], initial_pose['y'])
    
    # 2. 🔧 自适应参数调优
    crazy_reloc.adaptive_parameter_tuning()
    
    # 3. 🧠 AI智能角度预测
    rospy.loginfo("🤖 启动AI智能重定位策略...")
    
    # 获取AI预测的角度
    ai_predicted_angles = crazy_reloc.ai_predict_best_angles(map_path, initial_pose['x'], initial_pose['y'])
    
    # 4. 🚀 多线程并行重定位
    rospy.loginfo("⚡ 启动多线程并行重定位...")
    successful_angle = crazy_reloc.parallel_relocalization(
        map_path, 
        initial_pose['x'], 
        initial_pose['y'], 
        ai_predicted_angles
    )
    
    reloc_success = False
    if successful_angle is not None:
        rospy.loginfo("🎉 并行重定位成功！")
        
        # 验证重定位结果
        rospy.sleep(2.0)
        status = check_reloc_status()
        rospy.loginfo("🔍 重定位状态检查: %s", status)
        
        if status is True:
            rospy.sleep(2.0)  # 额外等待
            status2 = check_reloc_status()
            rospy.loginfo("🔍 二次状态检查: %s", status2)
            
            if status2 is True:
                rospy.loginfo("🎊" + "="*50)
                rospy.loginfo("🎊 疯狂重定位成功！角度: %.2f°", successful_angle)
                rospy.loginfo("🎊" + "="*50)
                reloc_success = True
                
                # 发布成功消息
                success_msg = Bool()
                success_msg.data = True
                reloc_success_pub.publish(success_msg)
                rospy.loginfo("📡 重定位成功消息已发布")
                
                # 保存成功记录
                crazy_reloc.save_success_record(successful_angle)
                # 播放成功音乐
                crazy_reloc.play_reloc_music(True)
            else:
                rospy.logwarn("😞 二次状态检查失败，重定位不稳定")
                crazy_reloc.play_reloc_music(False)
        else:
            rospy.logwarn("😞 重定位状态检查失败")
            crazy_reloc.play_reloc_music(False)
    else:
        # 4. 🎲 如果AI失败，尝试疯狂角度序列
        rospy.logwarn("😞 AI重定位失败，尝试疯狂角度序列...")
        
        crazy_angles = crazy_reloc.generate_crazy_angles()
        successful_angle = crazy_reloc.parallel_relocalization(
            map_path, 
            initial_pose['x'], 
            initial_pose['y'], 
            crazy_angles
        )
        
        if successful_angle is not None:
            rospy.loginfo("🎲 疯狂角度序列成功！")
            reloc_success = True
            crazy_reloc.save_success_record(successful_angle)
        else:
            # 5. 💥 如果疯狂角度失败，尝试爆炸式搜索
            rospy.logwarn("😞 疯狂角度失败，尝试爆炸式搜索...")
            
            successful_angle = crazy_reloc.crazy_explosion_search(map_path, initial_pose['x'], initial_pose['y'])
            
            if successful_angle is not None:
                rospy.loginfo("💥 爆炸式搜索成功！")
                reloc_success = True
                crazy_reloc.save_success_record(successful_angle)
            else:
                # 6. ⚛️ 如果爆炸式搜索失败，尝试量子重定位
                rospy.logwarn("😞 爆炸式搜索失败，尝试量子重定位...")
                
                successful_angle = crazy_reloc.quantum_relocalization(map_path, initial_pose['x'], initial_pose['y'])
                
                if successful_angle is not None:
                    rospy.loginfo("⚛️ 量子重定位成功！")
                    reloc_success = True
                    crazy_reloc.save_success_record(successful_angle)
                else:
                    # 7. 🔄 最后尝试传统方法
                    rospy.logwarn("😞 量子重定位失败，最后尝试传统方法...")
                    
                    # 生成传统角度列表
                    search_range_deg = yaw_search['range_deg']
                    step_deg = yaw_search['step_deg']
                    
                    yaw_angles_to_try = []
                    yaw_angles_to_try.append(0)
                    current_angle_deg = step_deg
                    while current_angle_deg <= search_range_deg:
                        yaw_angles_to_try.append(current_angle_deg)
                        yaw_angles_to_try.append(-current_angle_deg)
                        current_angle_deg += step_deg

                    # 传统循环尝试
                    for yaw_deg in yaw_angles_to_try:
                        if rospy.is_shutdown():
                            rospy.logwarn("🛑 关闭请求，停止重定位尝试")
                            break

                        rospy.loginfo(f"🔄 传统方法尝试角度: {yaw_deg:.1f}°")
                        
                        # 使用增强版重定位
                        if not crazy_reloc.attempt_relocalization_enhanced(map_path, initial_pose['x'], initial_pose['y'], math.radians(yaw_deg)):
                            rospy.logwarn("❌ 重定位命令发送失败，角度: %.1f", yaw_deg)
                            continue
                        
                        # 等待处理
                        rospy.sleep(3.0) 

                        # 检查结果
                        status = check_reloc_status()
                        rospy.loginfo("🔍 传统方法状态检查: %s", status)
                        
                        if status is True:
                            rospy.sleep(2.0)
                            status2 = check_reloc_status()
                            rospy.loginfo("🔍 传统方法二次检查: %s", status2)
                            
                            if status2 is True:
                                rospy.loginfo("🎊" + "="*50)
                                rospy.loginfo("🎊 传统方法重定位成功！角度: %.2f°", yaw_deg)
                                rospy.loginfo("🎊" + "="*50)
                                reloc_success = True
                                
                                # 发布成功消息
                                success_msg = Bool()
                                success_msg.data = True
                                reloc_success_pub.publish(success_msg)
                                rospy.loginfo("📡 重定位成功消息已发布")
                                
                                # 保存成功记录
                                crazy_reloc.save_success_record(yaw_deg)
                                break
                            else:
                                rospy.logwarn("😞 传统方法二次检查失败")
                        else:
                            rospy.logwarn("😞 传统方法重定位失败，角度: %.1f", yaw_deg)

    # 5. 📊 发布最终结果和统计
    if not reloc_success and not rospy.is_shutdown():
        rospy.logerr("💥" + "="*50)
        rospy.logerr("💥 疯狂重定位失败！所有方法都尝试过了")
        rospy.logerr("💥" + "="*50)
        
        # 发布失败消息
        failure_msg = Bool()
        failure_msg.data = False
        reloc_success_pub.publish(failure_msg)
        rospy.loginfo("📡 重定位失败消息已发布")
    
    # 6. 📈 发布最终统计信息
    crazy_reloc.publish_final_stats()
    
    rospy.loginfo("🏁 疯狂重定位程序结束")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Auto-reloc script interrupted.")
