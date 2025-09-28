#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
服务状态清理脚本
用于清理可能残留的舞蹈服务状态，确保下次运行时状态干净
"""

import rospy
import subprocess
import sys
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID

def cleanup_services():
    """清理所有相关服务状态"""
    rospy.loginfo("开始清理服务状态...")
    
    try:
        # 1. 发布停止命令
        rospy.loginfo("发布停止命令...")
        cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        stop_msg = Twist()
        cmd_vel_pub.publish(stop_msg)
        time.sleep(0.5)
        
        # 2. 取消所有导航目标
        rospy.loginfo("取消所有导航目标...")
        cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        cancel_msg = GoalID()
        cancel_pub.publish(cancel_msg)
        time.sleep(0.5)
        
        # 3. 发布舞蹈停止命令
        rospy.loginfo("发布舞蹈停止命令...")
        dance_direction_pub = rospy.Publisher("dance_direction", String, queue_size=1)
        stop_dance_msg = String()
        stop_dance_msg.data = "stop"
        dance_direction_pub.publish(stop_dance_msg)
        time.sleep(0.5)
        
        # 4. 尝试调用舞蹈服务来重置状态
        rospy.loginfo("尝试重置舞蹈服务状态...")
        try:
            from std_srvs.srv import Trigger
            rospy.wait_for_service("play_dance", timeout=2.0)
            play_dance_service = rospy.ServiceProxy("play_dance", Trigger)
            # 发送一个停止请求来重置服务状态
            rospy.loginfo("调用舞蹈服务重置...")
        except rospy.ROSException:
            rospy.loginfo("舞蹈服务不可用，跳过重置")
        except Exception as e:
            rospy.logwarn(f"重置舞蹈服务失败: {e}")
        
        # 5. 尝试重置Up/Down服务
        for service_name in ["play_up", "play_down"]:
            try:
                rospy.loginfo(f"尝试重置{service_name}服务...")
                rospy.wait_for_service(service_name, timeout=2.0)
                service_proxy = rospy.ServiceProxy(service_name, Trigger)
                rospy.loginfo(f"{service_name}服务重置完成")
            except rospy.ROSException:
                rospy.loginfo(f"{service_name}服务不可用，跳过重置")
            except Exception as e:
                rospy.logwarn(f"重置{service_name}服务失败: {e}")
        
        rospy.loginfo("? 服务状态清理完成")
        
    except Exception as e:
        rospy.logerr(f"清理服务状态失败: {e}")

def kill_related_processes():
    """杀死可能残留的相关进程"""
    rospy.loginfo("检查并清理相关进程...")
    
    try:
        # 查找并杀死可能残留的new_plan进程
        result = subprocess.run(['pgrep', '-f', 'new_plan.py'], 
                              capture_output=True, text=True)
        if result.stdout.strip():
            pids = result.stdout.strip().split('\n')
            for pid in pids:
                if pid:
                    rospy.loginfo(f"杀死残留的new_plan进程: {pid}")
                    subprocess.run(['kill', '-9', pid])
                    time.sleep(0.5)
        
        # 查找并杀死可能残留的g1_client进程
        result = subprocess.run(['pgrep', '-f', 'g1_client'], 
                              capture_output=True, text=True)
        if result.stdout.strip():
            pids = result.stdout.strip().split('\n')
            for pid in pids:
                if pid:
                    rospy.loginfo(f"杀死残留的g1_client进程: {pid}")
                    subprocess.run(['kill', '-9', pid])
                    time.sleep(0.5)
        
        rospy.loginfo("? 进程清理完成")
        
    except Exception as e:
        rospy.logwarn(f"进程清理失败: {e}")

def main():
    """主函数"""
    rospy.init_node('service_cleanup', anonymous=True)
    
    rospy.loginfo("=== 服务状态清理脚本 ===")
    
    # 清理相关进程
    kill_related_processes()
    
    # 等待ROS系统稳定
    time.sleep(2.0)
    
    # 清理服务状态
    cleanup_services()
    
    rospy.loginfo("=== 清理完成，可以重新运行new_plan ===")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("清理脚本被中断")
    except Exception as e:
        rospy.logerr(f"清理脚本执行失败: {e}")
        sys.exit(1)
