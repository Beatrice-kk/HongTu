#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
G1机器人模式切换工具
用于强制切换机器人到主运控模式
"""

import time
import sys
import os

class RobotModeSwitcher:
    """机器人模式切换器"""
    
    def __init__(self):
        self.loco_client = None
        self.attempts = 0
        self.max_attempts = 5
    
    def init_client(self):
        """初始化运控客户端"""
        try:
            from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
            self.loco_client = LocoClient()
            self.loco_client.SetTimeout(5.0)
            self.loco_client.Init()
            print("? 运控客户端初始化成功")
            return True
        except Exception as e:
            print(f"? 运控客户端初始化失败: {e}")
            return False
    
    def get_current_mode(self):
        """获取当前模式"""
        if self.loco_client is None:
            return -1, "客户端未初始化"
        
        try:
            code, mode_id = self.loco_client.GetFsmId()
            return code, mode_id
        except Exception as e:
            return -1, f"获取模式失败: {e}"
    
    def is_main_loco_mode(self, mode_id):
        """检查是否为主运控模式"""
        main_loco_modes = [500, 501, 801]  # 主运控模式ID
        return mode_id in main_loco_modes
    
    def wait_for_main_loco_mode(self):
        """等待机器人进入主运控模式"""
        print("? 等待机器人进入主运控模式...")
        
        while self.attempts < self.max_attempts:
            self.attempts += 1
            print(f"? 尝试 {self.attempts}/{self.max_attempts}...")
            
            code, mode_id = self.get_current_mode()
            
            if code == 0:
                mode_names = {
                    0: "零力矩模式",
                    1: "阻尼模式",
                    2: "位控下蹲",
                    3: "位控落座",
                    4: "锁定站立",
                    500: "常规运控（主运控）",
                    501: "常规运控-3Dof-waist",
                    801: "走跑运控"
                }
                
                mode_name = mode_names.get(mode_id, f"未知模式({mode_id})")
                print(f"? 当前模式: {mode_name}")
                
                if self.is_main_loco_mode(mode_id):
                    print("? 机器人已进入主运控模式！")
                    return True
                else:
                    print("? 机器人未处于主运控模式")
                    print("? 请使用遥控器切换到站立模式")
                    print("? 或者重启机器人")
            else:
                print(f"? 获取模式失败，错误码: {code}")
            
            if self.attempts < self.max_attempts:
                print("? 等待30秒后重试...")
                time.sleep(30)
        
        print("? 无法让机器人进入主运控模式")
        return False
    
    def run_mode_switch(self):
        """运行模式切换"""
        print("? G1机器人模式切换工具")
        print("=" * 50)
        
        # 初始化客户端
        if not self.init_client():
            print("? 无法初始化运控客户端")
            print("? 请检查:")
            print("   ? 机器人是否已开机")
            print("   ? 网络连接是否正常")
            print("   ? 机器人是否处于站立状态")
            return False
        
        # 等待主运控模式
        if self.wait_for_main_loco_mode():
            print("? 模式切换成功！")
            print("? 机器人现在应该可以正常控制移动了")
            return True
        else:
            print("? 模式切换失败")
            print("? 请尝试以下方法:")
            print("   1. 使用遥控器切换到站立模式")
            print("   2. 重启机器人")
            print("   3. 检查机器人是否正常开机")
            return False


def main():
    """主函数"""
    print("? 启动机器人模式切换工具...")
    
    try:
        switcher = RobotModeSwitcher()
        success = switcher.run_mode_switch()
        
        if success:
            print("\n? 机器人模式切换完成！")
            print("? 现在可以正常控制机器人移动了")
        else:
            print("\n? 机器人模式切换失败")
            print("? 请按照上述建议进行手动修复")
            
    except KeyboardInterrupt:
        print("\n?? 模式切换被用户中断")
    except Exception as e:
        print(f"? 模式切换过程中出现错误: {e}")


if __name__ == "__main__":
    main()
