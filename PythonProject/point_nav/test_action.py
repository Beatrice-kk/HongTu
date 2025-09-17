import rospy
import math
import tf.transformations as tft
import actionlib
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import PoseStamped, Pose
from typing import Optional
import sys
import threading
import time
import importlib.util
import os

# 添加SDK路径
sys.path.append("/home/unitree/unitree_sdk2_python/example/g1/high_level")
import g1_client_cwk

# 全局变量，用于存储遥控器实例引用
remote_controller_instance = None

# 用于表示g1_client_cwk.main函数是否已经启动的标志
main_started = False
main_ready = False

def simulate_button_press(remote, button_name, duration=0.2):
    """模拟单个按键按下"""
    print(f"🎮 模拟按下 {button_name} 按键")
    
    # 设置按下状态
    setattr(remote, button_name, 1)
    
    # 构造无线遥控器数据
    wireless_data = [0] * 4
    
    # 根据按键类型设置相应的位
    if button_name in ['R1', 'L1', 'Start', 'Select', 'R2', 'L2', 'F1']:
        bit_position = {'R1': 0, 'L1': 1, 'Start': 2, 'Select': 3, 
                        'R2': 4, 'L2': 5, 'F1': 6}.get(button_name)
        wireless_data[2] |= (1 << bit_position)
    elif button_name in ['A', 'B', 'X', 'Y', 'Up', 'Right', 'Down', 'Left']:
        bit_position = {'A': 0, 'B': 1, 'X': 2, 'Y': 3, 
                        'Up': 4, 'Right': 5, 'Down': 6, 'Left': 7}.get(button_name)
        wireless_data[3] |= (1 << bit_position)
    
    # 调用解析方法
    remote.parse(wireless_data)
    
    # 保持按键状态一段时间
    time.sleep(duration)
    
    # 释放按键
    setattr(remote, button_name, 0)
    
    # 清除无线遥控器状态
    wireless_data = [0] * 4
    remote.parse(wireless_data)
    
    print(f"🎮 释放 {button_name} 按键")

def simulate_combo_press(remote, button1, button2, duration=1.2):
    """模拟组合键按下"""
    print(f"🎮 模拟按下组合键 {button1} + {button2}")
    
    # 设置按下状态
    setattr(remote, button1, 1)
    setattr(remote, button2, 1)
    
    # 构造无线遥控器数据
    wireless_data = [0] * 4
    
    # 根据按键类型设置相应的位
    for button_name in [button1, button2]:
        if button_name in ['R1', 'L1', 'Start', 'Select', 'R2', 'L2', 'F1']:
            bit_position = {'R1': 0, 'L1': 1, 'Start': 2, 'Select': 3, 
                            'R2': 4, 'L2': 5, 'F1': 6}.get(button_name)
            wireless_data[2] |= (1 << bit_position)
        elif button_name in ['A', 'B', 'X', 'Y', 'Up', 'Right', 'Down', 'Left']:
            bit_position = {'A': 0, 'B': 1, 'X': 2, 'Y': 3, 
                            'Up': 4, 'Right': 5, 'Down': 6, 'Left': 7}.get(button_name)
            wireless_data[3] |= (1 << bit_position)
    
    # 调用解析方法
    remote.parse(wireless_data)
    
    # 保持按键状态一段时间
    time.sleep(duration)
    
    # 释放按键
    setattr(remote, button1, 0)
    setattr(remote, button2, 0)
    
    # 清除无线遥控器状态
    wireless_data = [0] * 4
    remote.parse(wireless_data)
    
    print(f"🎮 释放组合键 {button1} + {button2}")

# 修改 RemoteController 类，用于捕获实例
class RemoteControllerCapturer(g1_client_cwk.RemoteController):
    def __init__(self):
        super().__init__()
        global remote_controller_instance
        remote_controller_instance = self
        print("✅ 已捕获 RemoteController 实例")

# 替换原始的 RemoteController 类
g1_client_cwk.RemoteController = RemoteControllerCapturer

# 用于运行主函数的线程函数
def run_main():
    global main_started, main_ready
    main_started = True
    
    # 确保系统参数列表包含网卡参数
    if len(sys.argv) < 2:
        sys.argv.append("lo")  # 默认使用lo网卡
    
    try:
        print("🚀 启动 g1_client_cwk.main() 函数...")
        result = g1_client_cwk.main()
        print("⚠️ g1_client_cwk.main() 函数已退出，返回值:", result)
    except Exception as e:
        print(f"❌ g1_client_cwk.main() 函数执行出错: {e}")
        import traceback
        traceback.print_exc()
    finally:
        main_started = False

# 主程序
if __name__ == "__main__":
    try:
        # 创建并启动主线程
        main_thread = threading.Thread(target=run_main)
        main_thread.daemon = True  # 设置为守护线程，这样主线程退出时，它也会退出
        main_thread.start()
        
        print("⏳ 等待 RemoteController 实例被捕获...")
        # 等待RemoteController实例被捕获
        timeout = 30  # 设置30秒超时
        start_time = time.time()
        while remote_controller_instance is None:
            time.sleep(0.5)
            if time.time() - start_time > timeout:
                print("❌ 等待 RemoteController 实例超时")
                sys.exit(1)
            if not main_started:
                print("❌ 主线程已退出，无法获取 RemoteController 实例")
                sys.exit(1)
        
        # 给系统一些时间进行初始化
        print("✅ 已获取 RemoteController 实例")
        print("⏳ 等待8秒，确保系统初始化...")
        time.sleep(8)
        
        # 激活功能
        print("🔄 激活功能 (F1+Start)...")
        simulate_combo_press(remote_controller_instance, 'F1', 'Start')
        time.sleep(2)  # 等待功能激活
        
        # 现在可以模拟按键操作
        print("🔄 模拟按下F1    +   X 按键...")
        simulate_combo_press(remote_controller_instance, 'F1','X')
        time.sleep(1)
        
      #   # 模拟组合键操作 - 播放X动作
      #   print("🔄 模拟组合键 L1+X 播放X动作...")
      #   simulate_combo_press(remote_controller_instance, 'L1', 'X')
      #   time.sleep(3)  # 等待动作播放
        
      #   # 取消播放
      #   print("🔄 模拟组合键 L1+F1 取消播放...")
      #   simulate_combo_press(remote_controller_instance, 'L1', 'F1')
      #   time.sleep(2)
        
      #   # 取消激活功能
      #   print("🔄 模拟组合键 F1+Select 取消激活功能...")
      #   simulate_combo_press(remote_controller_instance, 'F1', 'Select')
        
        print("✅ 按键模拟测试完成")
        
        # 让程序继续运行，直到用户中断
        print("程序将继续运行。按 Ctrl+C 退出...")
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n👋 收到中断信号，准备退出")
    except Exception as e:
        print(f"\n❌ 程序运行出错: {e}")
        import traceback
        traceback.print_exc()