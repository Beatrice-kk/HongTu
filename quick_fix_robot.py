#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
G1机器人快速修复工具
用于快速解决机器人"控制不了下肢移动"的问题
"""

import time
import subprocess
import sys
import os

class QuickRobotFix:
    """机器人快速修复工具"""
    
    def __init__(self):
        self.fix_attempts = 0
        self.max_attempts = 3
    
    def check_robot_status(self):
        """检查机器人状态"""
        print("? 检查机器人状态...")
        
        try:
            # 检查机器人是否开机
            result = subprocess.run(['ping', '-c', '1', '192.168.123.161'], 
                                 capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                print("? 机器人网络连接正常")
                return True
            else:
                print("? 机器人网络连接异常")
                return False
        except Exception as e:
            print(f"? 检查机器人状态失败: {e}")
            return False
    
    def kill_all_robot_processes(self):
        """关闭所有机器人相关进程"""
        print("? 关闭所有机器人相关进程...")
        
        processes_to_kill = [
            'g1_client',
            'new_plan',
            'g1_control',
            'move_base',
            'amcl',
            'map_server',
            'fastlio',
            'ai_sport'
        ]
        
        for process in processes_to_kill:
            try:
                subprocess.run(['pkill', '-f', process], check=False)
                print(f"? 已关闭 {process} 相关进程")
            except Exception as e:
                print(f"?? 关闭 {process} 失败: {e}")
        
        time.sleep(2)
    
    def restart_robot_services(self):
        """重启机器人服务"""
        print("? 重启机器人服务...")
        
        try:
            # 重启ai_sport服务
            subprocess.run(['sudo', 'systemctl', 'restart', 'ai_sport'], check=False)
            print("? 已重启 ai_sport 服务")
            time.sleep(3)
            
            # 重启其他可能需要的服务
            services = ['g1_control', 'move_base']
            for service in services:
                try:
                    subprocess.run(['sudo', 'systemctl', 'restart', service], check=False)
                    print(f"? 已重启 {service} 服务")
                except:
                    pass
                    
        except Exception as e:
            print(f"?? 重启服务失败: {e}")
    
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
    
    def run_quick_fix(self):
        """运行快速修复"""
        print("? 开始快速修复机器人...")
        print("=" * 50)
        
        # 步骤1: 检查机器人状态
        if not self.check_robot_status():
            print("? 机器人网络连接异常，请检查:")
            print("   1. 机器人是否已开机")
            print("   2. 网络连接是否正常")
            print("   3. IP地址是否正确")
            return False
        
        # 步骤2: 关闭所有相关进程
        self.kill_all_robot_processes()
        
        # 步骤3: 重启机器人服务
        self.restart_robot_services()
        
        # 步骤4: 启动G1客户端
        if self.start_g1_client():
            print("? 快速修复完成！")
            print("? 机器人应该已经可以正常控制移动了")
            return True
        else:
            print("? 快速修复失败")
            return False
    
    def run_manual_fix_guide(self):
        """运行手动修复指南"""
        print("\n? 手动修复指南:")
        print("=" * 50)
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


def main():
    """主函数"""
    print("? G1机器人快速修复工具")
    print("=" * 50)
    print("此工具用于修复机器人'控制不了下肢移动'的问题")
    print()
    
    fixer = QuickRobotFix()
    
    try:
        # 运行快速修复
        success = fixer.run_quick_fix()
        
        if not success:
            # 如果自动修复失败，显示手动修复指南
            fixer.run_manual_fix_guide()
        
    except KeyboardInterrupt:
        print("\n?? 修复被用户中断")
    except Exception as e:
        print(f"? 修复过程中出现错误: {e}")
        fixer.run_manual_fix_guide()


if __name__ == "__main__":
    main()
