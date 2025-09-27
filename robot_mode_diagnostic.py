#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
G1机器人模式诊断和修复工具
用于解决机器人进入"控制不了下肢移动"状态的问题
"""

import time
import sys
import os

class G1ModeDiagnostic:
    """G1机器人模式诊断器"""
    
    # 模式ID定义
    MODE_ZERO_TORQUE = 0      # 零力矩模式
    MODE_DAMP = 1             # 阻尼模式
    MODE_SQUAT_POS = 2        # 位控下蹲
    MODE_SIT_POS = 3          # 位控落座
    MODE_STAND_LOCK = 4       # 锁定站立
    MODE_BALANCE_SQUAT = 706  # 平衡下蹲
    MODE_STAND_UP = 500       # 常规运控（主运控）
    MODE_STAND_3DOF = 501     # 常规运控-3Dof-waist
    MODE_WALK_RUN = 801       # 走跑运控
    
    # 主运控模式列表
    MAIN_LOCO_MODES = [MODE_STAND_UP, MODE_STAND_3DOF, MODE_WALK_RUN]
    
    def __init__(self):
        """初始化诊断器"""
        self.loco_client = None
        self._last_mode_id = None
        
        try:
            from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
            self.loco_client = LocoClient()
            self.loco_client.SetTimeout(3.0)
            self.loco_client.Init()
            print("? 运控客户端初始化成功")
        except Exception as e:
            print(f"? 运控客户端初始化失败: {e}")
            print("? 请确保机器人已开机且网络连接正常")
    
    def get_current_mode(self):
        """获取当前机器人模式"""
        if self.loco_client is None:
            return -1, "客户端未初始化"
        
        try:
            code, mode_id = self.loco_client.GetFsmId()
            return code, mode_id
        except Exception as e:
            return -1, f"获取模式失败: {e}"
    
    def get_mode_name(self, mode_id):
        """获取模式名称"""
        mode_names = {
            self.MODE_ZERO_TORQUE: "零力矩模式",
            self.MODE_DAMP: "阻尼模式", 
            self.MODE_SQUAT_POS: "位控下蹲",
            self.MODE_SIT_POS: "位控落座",
            self.MODE_STAND_LOCK: "锁定站立",
            self.MODE_BALANCE_SQUAT: "平衡下蹲",
            self.MODE_STAND_UP: "常规运控（主运控）",
            self.MODE_STAND_3DOF: "常规运控-3Dof-waist",
            self.MODE_WALK_RUN: "走跑运控"
        }
        return mode_names.get(mode_id, f"未知模式({mode_id})")
    
    def is_main_loco_mode(self, mode_id):
        """检查是否为主运控模式"""
        return mode_id in self.MAIN_LOCO_MODES
    
    def diagnose_current_state(self):
        """诊断当前状态"""
        print("? 开始诊断机器人状态...")
        print("=" * 50)
        
        code, mode_id = self.get_current_mode()
        
        if code != 0:
            print(f"? 无法获取机器人模式，错误码: {code}")
            if code == 3102:
                print("? 可能原因: 网络连接问题或服务不可用")
            elif code == 3103:
                print("? 可能原因: API未注册，请检查服务是否正常运行")
            elif code == 3104:
                print("? 可能原因: 请求超时，请检查网络连接")
            elif code == 3202:
                print("? 可能原因: 服务端内部错误")
            elif code == 3203:
                print("? 可能原因: API在服务端未实现")
            elif code == 3205:
                print("? 可能原因: 请求被拒绝，可能需要更高权限")
            return False
        
        mode_name = self.get_mode_name(mode_id)
        is_main_loco = self.is_main_loco_mode(mode_id)
        
        print(f"? 当前模式: {mode_name}")
        print(f"? 模式ID: {mode_id}")
        
        if is_main_loco:
            print("? 机器人处于主运控模式，可以正常控制移动")
            return True
        else:
            print("? 机器人不处于主运控模式，无法控制移动")
            print("? 这就是为什么机器人动不了的原因！")
            return False
    
    def suggest_solutions(self, mode_id):
        """根据当前模式建议解决方案"""
        print("\n? 解决方案建议:")
        print("=" * 50)
        
        if mode_id == self.MODE_ZERO_TORQUE:
            print("? 当前处于零力矩模式，建议:")
            print("   1. 使用遥控器切换到站立模式")
            print("   2. 或者运行: python g1_client.py 切换到主运控模式")
            print("   3. 检查机器人是否已开机并站立")
            
        elif mode_id == self.MODE_DAMP:
            print("? 当前处于阻尼模式，建议:")
            print("   1. 使用遥控器切换到站立模式")
            print("   2. 或者运行: python g1_client.py 切换到主运控模式")
            
        elif mode_id == self.MODE_SQUAT_POS:
            print("? 当前处于位控下蹲模式，建议:")
            print("   1. 使用遥控器让机器人站起来")
            print("   2. 或者运行: python g1_client.py 切换到主运控模式")
            
        elif mode_id == self.MODE_SIT_POS:
            print("? 当前处于位控落座模式，建议:")
            print("   1. 使用遥控器让机器人站起来")
            print("   2. 或者运行: python g1_client.py 切换到主运控模式")
            
        elif mode_id == self.MODE_STAND_LOCK:
            print("? 当前处于锁定站立模式，建议:")
            print("   1. 使用遥控器解锁站立模式")
            print("   2. 或者运行: python g1_client.py 切换到主运控模式")
            
        elif mode_id == self.MODE_BALANCE_SQUAT:
            print("? 当前处于平衡下蹲模式，建议:")
            print("   1. 使用遥控器让机器人站起来")
            print("   2. 或者运行: python g1_client.py 切换到主运控模式")
            
        else:
            print("? 未知模式，建议:")
            print("   1. 重启机器人")
            print("   2. 检查机器人是否正常开机")
            print("   3. 运行: python g1_client.py 尝试切换到主运控模式")
    
    def try_switch_to_main_loco(self):
        """尝试切换到主运控模式"""
        print("\n? 尝试切换到主运控模式...")
        print("=" * 50)
        
        try:
            # 这里可以添加具体的模式切换逻辑
            # 由于SDK限制，通常需要通过遥控器或重启机器人来切换模式
            print("? 由于SDK限制，无法直接切换模式")
            print("? 请使用以下方法之一:")
            print("   1. 使用遥控器切换到站立模式")
            print("   2. 重启机器人")
            print("   3. 运行: python g1_client.py 并等待模式检查")
            
        except Exception as e:
            print(f"? 切换模式失败: {e}")
    
    def run_full_diagnostic(self):
        """运行完整诊断"""
        print("? G1机器人模式诊断工具")
        print("=" * 50)
        
        # 诊断当前状态
        is_ok = self.diagnose_current_state()
        
        if not is_ok:
            code, mode_id = self.get_current_mode()
            if code == 0:
                self.suggest_solutions(mode_id)
                self.try_switch_to_main_loco()
        
        print("\n? 诊断完成")
        print("=" * 50)
        
        if is_ok:
            print("? 机器人状态正常，可以正常控制移动")
        else:
            print("? 机器人状态异常，需要修复后才能控制移动")
            print("? 请按照上述建议进行修复")


def main():
    """主函数"""
    print("? 启动G1机器人模式诊断工具...")
    
    try:
        diagnostic = G1ModeDiagnostic()
        diagnostic.run_full_diagnostic()
    except KeyboardInterrupt:
        print("\n?? 诊断被用户中断")
    except Exception as e:
        print(f"? 诊断过程中出现错误: {e}")
        print("? 请检查机器人是否已开机且网络连接正常")


if __name__ == "__main__":
    main()
