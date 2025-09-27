#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
G1机器人移动控制修复工具
综合解决方案，用于修复机器人"控制不了下肢移动"的问题
"""

import time
import subprocess
import sys
import os
import signal

class RobotMovementFixer:
    """机器人移动控制修复器"""
    
    def __init__(self):
        self.loco_client = None
        self.fix_attempts = 0
        self.max_attempts = 3
        
    def print_banner(self):
        """打印横幅"""
        print("? G1机器人移动控制修复工具")
        print("=" * 60)
        print("此工具用于修复机器人'控制不了下肢移动'的问题")
        print("=" * 60)
        print()
    
    def check_robot_connection(self):
        """检查机器人连接"""
        print("? 检查机器人连接...")
        
        try:
            # 检查网络连接
            result = subprocess.run(['ping', '-c', '1', '192.168.123.161'], 
                                 capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                print("? 机器人网络连接正常")
                return True
            else:
                print("? 机器人网络连接异常")
                return False
        except Exception as e:
            print(f"? 检查连接失败: {e}")
            return False
    
    def kill_all_processes(self):
        """关闭所有相关进程"""
        print("? 关闭所有机器人相关进程...")
        
        processes = [
            'g1_client',
            'new_plan',
            'g1_control',
            'move_base',
            'amcl',
            'map_server',
            'fastlio',
            'ai_sport',
            'rosmaster',
            'rosout'
        ]
        
        for process in processes:
            try:
                subprocess.run(['pkill', '-f', process], check=False)
                print(f"? 已关闭 {process} 相关进程")
            except Exception as e:
                print(f"?? 关闭 {process} 失败: {e}")
        
        time.sleep(3)
    
    def restart_services(self):
        """重启服务"""
        print("? 重启机器人服务...")
        
        services = ['ai_sport', 'g1_control']
        for service in services:
            try:
                subprocess.run(['sudo', 'systemctl', 'restart', service], check=False)
                print(f"? 已重启 {service} 服务")
                time.sleep(2)
            except Exception as e:
                print(f"?? 重启 {service} 失败: {e}")
    
    def init_loco_client(self):
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
    
    def get_robot_mode(self):
        """获取机器人模式"""
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
    
    def get_mode_description(self, mode_id):
        """获取模式描述"""
        mode_descriptions = {
            0: "零力矩模式 - 机器人无法控制移动",
            1: "阻尼模式 - 机器人无法控制移动", 
            2: "位控下蹲 - 机器人无法控制移动",
            3: "位控落座 - 机器人无法控制移动",
            4: "锁定站立 - 机器人无法控制移动",
            500: "常规运控（主运控） - 可以正常控制移动",
            501: "常规运控-3Dof-waist - 可以正常控制移动",
            801: "走跑运控 - 可以正常控制移动"
        }
        return mode_descriptions.get(mode_id, f"未知模式({mode_id})")
    
    def diagnose_robot_mode(self):
        """诊断机器人模式"""
        print("? 诊断机器人模式...")
        
        code, mode_id = self.get_robot_mode()
        
        if code != 0:
            print(f"? 无法获取机器人模式，错误码: {code}")
            if code == 3102:
                print("? 可能原因: 网络连接问题或服务不可用")
            elif code == 3103:
                print("? 可能原因: API未注册，请检查服务是否正常运行")
            elif code == 3104:
                print("? 可能原因: 请求超时，请检查网络连接")
            return False
        
        mode_desc = self.get_mode_description(mode_id)
        is_main_loco = self.is_main_loco_mode(mode_id)
        
        print(f"? 当前模式: {mode_desc}")
        
        if is_main_loco:
            print("? 机器人处于主运控模式，可以正常控制移动")
            return True
        else:
            print("? 机器人不处于主运控模式，无法控制移动")
            print("? 这就是为什么机器人动不了的原因！")
            return False
    
    def wait_for_main_loco_mode(self):
        """等待机器人进入主运控模式"""
        print("? 等待机器人进入主运控模式...")
        
        attempts = 0
        max_attempts = 5
        
        while attempts < max_attempts:
            attempts += 1
            print(f"? 尝试 {attempts}/{max_attempts}...")
            
            code, mode_id = self.get_robot_mode()
            
            if code == 0:
                mode_desc = self.get_mode_description(mode_id)
                print(f"? 当前模式: {mode_desc}")
                
                if self.is_main_loco_mode(mode_id):
                    print("? 机器人已进入主运控模式！")
                    return True
                else:
                    print("? 机器人未处于主运控模式")
                    print("? 请使用遥控器切换到站立模式")
                    print("? 或者重启机器人")
            else:
                print(f"? 获取模式失败，错误码: {code}")
            
            if attempts < max_attempts:
                print("? 等待30秒后重试...")
                time.sleep(30)
        
        print("? 无法让机器人进入主运控模式")
        return False
    
    def start_g1_client(self):
        """启动G1客户端"""
        print("? 启动G1客户端...")
        
        try:
            # 切换到正确的目录
            os.chdir('/home/unitree/HongTu/unitree_sdk2_python/example/g1/high_level')
            
            # 启动G1客户端
            process = subprocess.Popen(['python3', 'g1_client_now.py'], 
                                     stdout=subprocess.PIPE, 
                                     stderr=subprocess.PIPE)
            
            print("? G1客户端已启动")
            print("? 请等待30秒让机器人切换到主运控模式...")
            
            # 等待一段时间让机器人切换模式
            time.sleep(30)
            
            # 检查进程是否还在运行
            if process.poll() is None:
                print("? G1客户端运行正常")
                return True
            else:
                print("? G1客户端启动失败")
                return False
                
        except Exception as e:
            print(f"? 启动G1客户端失败: {e}")
            return False
    
    def run_automatic_fix(self):
        """运行自动修复"""
        print("? 开始自动修复...")
        
        # 步骤1: 检查连接
        if not self.check_robot_connection():
            return False
        
        # 步骤2: 关闭所有进程
        self.kill_all_processes()
        
        # 步骤3: 重启服务
        self.restart_services()
        
        # 步骤4: 初始化客户端
        if not self.init_loco_client():
            return False
        
        # 步骤5: 诊断模式
        if self.diagnose_robot_mode():
            print("? 机器人已处于主运控模式，无需修复")
            return True
        
        # 步骤6: 启动G1客户端
        if self.start_g1_client():
            # 步骤7: 等待主运控模式
            if self.wait_for_main_loco_mode():
                print("? 自动修复成功！")
                return True
        
        print("? 自动修复失败")
        return False
    
    def run_manual_fix_guide(self):
        """运行手动修复指南"""
        print("\n? 手动修复指南:")
        print("=" * 60)
        print("如果自动修复失败，请按以下步骤手动修复:")
        print()
        print("1?? 使用遥控器修复:")
        print("   ? 按遥控器上的站立按钮")
        print("   ? 或者按遥控器上的模式切换按钮")
        print("   ? 确保机器人处于站立状态")
        print()
        print("2?? 重启机器人:")
        print("   ? 关闭机器人电源")
        print("   ? 等待10秒")
        print("   ? 重新开机")
        print("   ? 等待机器人完全启动")
        print()
        print("3?? 运行诊断工具:")
        print("   ? python3 robot_mode_diagnostic.py")
        print("   ? 查看当前机器人模式")
        print()
        print("4?? 启动控制程序:")
        print("   ? cd /home/unitree/HongTu/unitree_sdk2_python/example/g1/high_level")
        print("   ? python3 g1_client_now.py")
        print("   ? 等待模式检查完成")
        print()
        print("5?? 检查模式:")
        print("   ? 运行: python3 switch_robot_mode.py")
        print("   ? 确认机器人进入主运控模式")
    
    def run_full_fix(self):
        """运行完整修复"""
        self.print_banner()
        
        try:
            # 运行自动修复
            success = self.run_automatic_fix()
            
            if success:
                print("\n? 机器人移动控制修复成功！")
                print("? 机器人现在应该可以正常控制移动了")
                print("? 可以尝试使用遥控器或程序控制机器人移动")
            else:
                print("\n? 自动修复失败")
                print("? 请按照以下手动修复指南进行操作")
                self.run_manual_fix_guide()
                
        except KeyboardInterrupt:
            print("\n?? 修复被用户中断")
        except Exception as e:
            print(f"? 修复过程中出现错误: {e}")
            self.run_manual_fix_guide()


def main():
    """主函数"""
    try:
        fixer = RobotMovementFixer()
        fixer.run_full_fix()
    except Exception as e:
        print(f"? 程序运行失败: {e}")


if __name__ == "__main__":
    main()
