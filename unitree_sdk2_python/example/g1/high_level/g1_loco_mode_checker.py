#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
G1机器人运控模式检测器模块
"""

import time

class G1LocoModeChecker:
    """G1机器人主运控模式检测器"""
    
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
    
    def __init__(self, network_interface="eth0"):
        """
        初始化检测器
        
        Args:
            network_interface: 网络接口名称
        """
        # 创建客户端
        self.loco_client = None
        
        try:
            from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
            self.loco_client = LocoClient()
            self.loco_client.SetTimeout(3.0)  # 设置超时时间
            self.loco_client.Init()
        except Exception as e:
            print(f"??  初始化运控客户端失败: {e}")
    
    def GetFsmId(self):
        """
        获取当前机器人模式ID（带重试机制）
        
        Returns:
            tuple: (错误码, 模式ID)
        """
        if self.loco_client is None:
            return -1, 0
            
        try:
            # 直接调用底层API获取FSM ID
            from unitree_sdk2py.g1.loco.g1_loco_api import ROBOT_API_ID_LOCO_GET_FSM_ID
            import json
            code, data = self.loco_client._Call(ROBOT_API_ID_LOCO_GET_FSM_ID, "{}")
            
            if code == 0 and data:
                result = json.loads(data)
                mode_id = result.get("data", 0)
                return code, mode_id
            return code, 0
        except Exception as e:
            print(f"? 调用GetFsmId API时出错: {e}")
            return -1, 0
    
    def is_in_main_loco_mode(self):
        """
        检查机器人是否处于主运控模式
        
        Returns:
            bool: 如果处于主运控模式返回True，否则返回False
        """
        if self.loco_client is None:
            return False  # 如果无法检测，则返回False
            
        # 获取当前模式ID
        code, mode_id = self.GetFsmId()
        
        if code != 0:
            # 减少错误信息输出频率，只在必要时打印
            if not hasattr(self, '_last_error_time'):
                self._last_error_time = 0
                
            current_time = time.time()
            # 每隔5秒以上才打印一次错误信息
            if current_time - self._last_error_time > 5:
                error_messages = {
                    3102: "请求发送错误，可能是网络连接问题或服务不可用",
                    3103: "API未注册，请检查服务是否正常运行",
                    3104: "请求超时，请检查网络连接",
                    3202: "服务端内部错误",
                    3203: "API在服务端未实现",
                    3205: "请求被拒绝，可能需要更高权限"
                }
                error_desc = error_messages.get(code, "未知错误")
                print(f"??  获取机器人模式失败，错误码: {code} ({error_desc})")
                self._last_error_time = current_time
            return False
            
        # 检查是否为主运控模式
        is_main_loco = mode_id in self.MAIN_LOCO_MODES
        
        # 只在模式发生变化时打印信息
        if not hasattr(self, '_last_mode_id') or self._last_mode_id != mode_id:
            self._last_mode_id = mode_id
            if is_main_loco:
                mode_names = {
                    self.MODE_STAND_UP: "常规运控（主运控）",
                    self.MODE_STAND_3DOF: "常规运控-3Dof-waist",
                    self.MODE_WALK_RUN: "走跑运控"
                }
                print(f"? 机器人处于主运控模式: {mode_names.get(mode_id, '未知主运控模式')}")
            else:
                mode_names = {
                    self.MODE_ZERO_TORQUE: "零力矩模式",
                    self.MODE_DAMP: "阻尼模式",
                    self.MODE_SQUAT_POS: "位控下蹲",
                    self.MODE_SIT_POS: "位控落座",
                    self.MODE_STAND_LOCK: "锁定站立",
                    self.MODE_BALANCE_SQUAT: "平衡下蹲"
                }
                print(f"? 机器人不处于主运控模式: {mode_names.get(mode_id, '其他模式')}")
            
        return is_main_loco
