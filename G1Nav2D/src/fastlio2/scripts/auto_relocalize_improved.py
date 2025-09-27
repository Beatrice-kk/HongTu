#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import yaml
import os
import sys
import time
import logging
from datetime import datetime
from fastlio.srv import SlamReLoc, SlamReLocRequest
from fastlio.srv import SlamRelocCheck, SlamRelocCheckRequest
from std_srvs.srv import Empty

class AutoRelocalizer:
    def __init__(self):
        """初始化自动重定位器"""
        rospy.init_node('one_key_reloc_node', anonymous=True)
        
        # 设置日志
        self.setup_logging()
        
        # 初始化状态
        self.reloc_success = False
        self.max_attempts = 0
        self.attempt_count = 0
        
        # 服务超时设置
        self.service_timeout = 10.0
        self.status_check_interval = 1.0
        
        self.logger.info("自动重定位器初始化完成")
    
    def setup_logging(self):
        """设置日志记录"""
        # 创建日志目录
        log_dir = "/home/unitree/HongTu/PythonProject/point_nav/logs"
        os.makedirs(log_dir, exist_ok=True)
        
        # 设置日志文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_file = os.path.join(log_dir, f"auto_reloc_{timestamp}.log")
        
        # 配置日志
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(log_file),
                logging.StreamHandler(sys.stdout)
            ]
        )
        self.logger = logging.getLogger(__name__)
        self.logger.info(f"日志文件: {log_file}")
    
    def wait_for_services(self, timeout=30.0):
        """等待必要的服务启动"""
        required_services = [
            '/slam_reloc',
            '/slam_reloc_check'
        ]
        
        self.logger.info("等待必要服务启动...")
        start_time = time.time()
        
        for service_name in required_services:
            try:
                self.logger.info(f"等待服务: {service_name}")
                rospy.wait_for_service(service_name, timeout=timeout)
                self.logger.info(f"服务 {service_name} 已就绪")
            except rospy.ROSException as e:
                self.logger.error(f"服务 {service_name} 启动超时: {e}")
                return False
        
        # 额外等待时间确保服务完全就绪
        self.logger.info("服务就绪，等待系统稳定...")
        rospy.sleep(3.0)
        return True
    
    def call_service(self, service_name, service_type, request=None, timeout=None):
        """增强的服务调用函数"""
        if timeout is None:
            timeout = self.service_timeout
            
        self.logger.debug(f"调用服务: {service_name}")
        
        try:
            rospy.wait_for_service(service_name, timeout=timeout)
        except rospy.ROSException:
            self.logger.error(f"服务 '{service_name}' 不可用")
            return None
        
        try:
            service_proxy = rospy.ServiceProxy(service_name, service_type)
            if request is None and service_type == Empty:
                response = service_proxy()
            else:
                response = service_proxy(request)
            return response
        except rospy.ServiceException as e:
            self.logger.error(f"服务调用失败 '{service_name}': {e}")
            return None
        except Exception as e:
            self.logger.error(f"服务调用异常 '{service_name}': {e}")
            return None
    
    def attempt_relocalization(self, map_path, x, y, yaw_rad):
        """尝试重定位"""
        self.logger.info(f"尝试重定位 - 位置: ({x:.2f}, {y:.2f}), 朝向: {math.degrees(yaw_rad):.1f}°")
        
        req = SlamReLocRequest()
        req.pcd_path = map_path
        req.x = x
        req.y = y
        req.z = 0.0
        req.roll = 0.0
        req.pitch = 0.0
        req.yaw = yaw_rad

        response = self.call_service('/slam_reloc', SlamReLoc, req)
        if response is None:
            self.logger.warning("重定位请求发送失败")
            return False
            
        success = response.status == 1
        self.logger.info(f"重定位请求结果: {'成功' if success else '失败'}")
        return success
    
    def check_reloc_status(self, max_checks=5):
        """检查重定位状态，支持多次检查"""
        for i in range(max_checks):
            self.logger.debug(f"检查重定位状态 ({i+1}/{max_checks})")
            
            response = self.call_service('/slam_reloc_check', SlamRelocCheck, SlamRelocCheckRequest())
            if response is None:
                self.logger.warning(f"状态检查失败 ({i+1}/{max_checks})")
                if i < max_checks - 1:
                    rospy.sleep(self.status_check_interval)
                continue
            
            if response.status:
                self.logger.info("重定位状态检查: 成功")
                return True
            else:
                self.logger.debug("重定位状态检查: 失败")
                if i < max_checks - 1:
                    rospy.sleep(self.status_check_interval)
        
        self.logger.warning("重定位状态检查: 多次检查均失败")
        return False
    
    def load_config(self):
        """加载配置参数"""
        try:
            config = rospy.get_param('~')
            self.map_path = config['map_pcd_path']
            self.initial_pose = config['initial_pose']
            self.yaw_search = config['yaw_search']
            
            # 验证配置
            if not os.path.exists(self.map_path):
                self.logger.error(f"地图文件不存在: {self.map_path}")
                return False
                
            self.logger.info("配置加载成功:")
            self.logger.info(f"  地图文件: {self.map_path}")
            self.logger.info(f"  初始位置: ({self.initial_pose['x']:.2f}, {self.initial_pose['y']:.2f})")
            self.logger.info(f"  搜索范围: {self.yaw_search['range_deg']}°")
            self.logger.info(f"  搜索步长: {self.yaw_search['step_deg']}°")
            
            return True
            
        except KeyError as e:
            self.logger.error(f"配置参数缺失: {e}")
            self.logger.error("请使用: rosparam load your_config.yaml")
            return False
        except Exception as e:
            self.logger.error(f"配置加载失败: {e}")
            return False
    
    def generate_yaw_angles(self):
        """生成要搜索的Yaw角度列表"""
        search_range_deg = self.yaw_search['range_deg']
        step_deg = self.yaw_search['step_deg']
        
        yaw_angles = []
        # 从0度开始，双向扩展搜索
        yaw_angles.append(0)
        current_angle_deg = step_deg
        
        while current_angle_deg <= search_range_deg:
            yaw_angles.append(current_angle_deg)
            yaw_angles.append(-current_angle_deg)
            current_angle_deg += step_deg
        
        self.max_attempts = len(yaw_angles)
        self.logger.info(f"生成 {self.max_attempts} 个搜索角度")
        return yaw_angles
    
    def run_relocalization(self):
        """执行重定位流程"""
        self.logger.info("=" * 60)
        self.logger.info("开始自动重定位流程")
        self.logger.info("=" * 60)
        
        # 1. 等待服务启动
        if not self.wait_for_services():
            self.logger.error("服务启动失败，退出")
            return False
        
        # 2. 加载配置
        if not self.load_config():
            self.logger.error("配置加载失败，退出")
            return False
        
        # 3. 生成搜索角度
        yaw_angles = self.generate_yaw_angles()
        
        # 4. 执行重定位
        for yaw_deg in yaw_angles:
            if rospy.is_shutdown():
                self.logger.warning("收到关闭信号，停止重定位")
                break
            
            self.attempt_count += 1
            self.logger.info(f"尝试 {self.attempt_count}/{self.max_attempts}: 角度 {yaw_deg:.1f}°")
            
            # 发送重定位请求
            if not self.attempt_relocalization(
                self.map_path, 
                self.initial_pose['x'], 
                self.initial_pose['y'], 
                math.radians(yaw_deg)
            ):
                self.logger.warning(f"重定位请求失败，跳过角度 {yaw_deg:.1f}°")
                continue
            
            # 等待处理
            rospy.sleep(1.0)
            
            # 检查结果
            if self.check_reloc_status():
                self.logger.info("=" * 60)
                self.logger.info(f"重定位成功! 角度: {yaw_deg:.1f}°")
                self.logger.info("=" * 60)
                self.reloc_success = True
                return True
            else:
                self.logger.warning(f"重定位失败，角度 {yaw_deg:.1f}°，尝试下一个角度")
        
        # 所有角度都失败
        if not self.reloc_success and not rospy.is_shutdown():
            self.logger.error("=" * 60)
            self.logger.error("重定位失败: 已尝试所有角度")
            self.logger.error("=" * 60)
        
        return self.reloc_success

def main():
    try:
        reloc = AutoRelocalizer()
        success = reloc.run_relocalization()
        
        if success:
            rospy.loginfo("自动重定位完成")
        else:
            rospy.logerr("自动重定位失败")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("自动重定位脚本被中断")
    except Exception as e:
        rospy.logerr(f"自动重定位脚本异常: {e}")

if __name__ == '__main__':
    main()
