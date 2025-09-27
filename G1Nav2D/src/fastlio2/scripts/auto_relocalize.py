#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import yaml
from fastlio.srv import SlamReLoc, SlamReLocRequest
from fastlio.srv import SlamRelocCheck, SlamRelocCheckRequest
from std_srvs.srv import Empty
from std_msgs.msg import Bool

# --- 服务调用封装 ---

def call_service(service_name, service_type, request=None):
    """一个通用的服务调用函数"""
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

def attempt_relocalization(map_path, x, y, yaw_rad):
    """尝试使用给定的位姿进行一次重定位"""
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

# --- 主逻辑 ---

def main():
    rospy.init_node('one_key_reloc_node', anonymous=True)
    
    # 创建发布者，用于发布重定位成功消息
    reloc_success_pub = rospy.Publisher('/relocalization_success', Bool, queue_size=1, latch=True)

    # 1. 从参数服务器加载配置
    # 使用 rosparam load 命令将 YAML 文件加载到参数服务器
    # rosparam load path/to/reloc_config.yaml
    try:
        config = rospy.get_param('~') # 从私有命名空间获取参数
        map_path = config['map_pcd_path']
        initial_pose = config['initial_pose']
        yaw_search = config['yaw_search']
    except KeyError as e:
        rospy.logerr("Configuration parameter %s not found on the parameter server. Did you load the YAML file?", e)
        rospy.logerr("Use: rosparam load your_config.yaml")
        return

    rospy.loginfo("Configuration loaded. Starting auto-relocalization...")
    rospy.loginfo("Map: %s", map_path)
    rospy.loginfo("Initial position (x,y): (%.2f, %.2f)", initial_pose['x'], initial_pose['y'])
    
    # 2. 生成要搜索的Yaw角度列表
    search_range_deg = yaw_search['range_deg']
    step_deg = yaw_search['step_deg']
    
    yaw_angles_to_try = []
    # 从0度开始，双向扩展搜索范围
    yaw_angles_to_try.append(0)
    current_angle_deg = step_deg
    while current_angle_deg <= search_range_deg:
        yaw_angles_to_try.append(current_angle_deg)
        yaw_angles_to_try.append(-current_angle_deg)
        current_angle_deg += step_deg

    # 3. 循环尝试重定位
    reloc_success = False
    for yaw_deg in yaw_angles_to_try:
        if rospy.is_shutdown():
            rospy.logwarn("Shutdown requested, stopping relocation attempts.")
            break

        # 发送重定位请求
        if not attempt_relocalization(map_path, initial_pose['x'], initial_pose['y'], math.radians(yaw_deg)):
            rospy.logwarn("Failed to send relocalization command for yaw %.1f. Skipping.", yaw_deg)
            continue
        
        # 等待更长时间让定位节点处理
        rospy.sleep(3.0) 

        # 检查结果
        status = check_reloc_status()
        rospy.loginfo("Reloc status check result: %s", status)
        
        # 再次等待并检查，确保重定位真正成功
        if status is True:
            rospy.sleep(2.0)  # 额外等待
            status2 = check_reloc_status()
            rospy.loginfo("Second reloc status check: %s", status2)
            
            if status2 is True:
                rospy.loginfo("======================================================")
                rospy.loginfo("SUCCESS! Relocalization successful at yaw: %.2f degrees", yaw_deg)
                rospy.loginfo("======================================================")
                reloc_success = True
                
                # 发布重定位成功消息
                success_msg = Bool()
                success_msg.data = True
                reloc_success_pub.publish(success_msg)
                rospy.loginfo("Published relocalization success message")
                
                break
            else:
                rospy.logwarn("Second status check failed. Relocalization not stable at yaw %.1f", yaw_deg)
        else:
            rospy.logwarn("Relocalization failed at yaw %.1f. Trying next angle.", yaw_deg)

    if not reloc_success and not rospy.is_shutdown():
        rospy.logerr("======================================================")
        rospy.logerr("FAILURE: Auto-relocalization failed after trying all angles.")
        rospy.logerr("======================================================")
        
        # 发布重定位失败消息
        failure_msg = Bool()
        failure_msg.data = False
        reloc_success_pub.publish(failure_msg)
        rospy.loginfo("Published relocalization failure message")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Auto-reloc script interrupted.")
