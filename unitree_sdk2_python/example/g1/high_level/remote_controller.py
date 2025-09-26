#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
G1机器人遥控器解析模块
"""

class RemoteController:
    """G1机器人遥控器解析类"""
    
    def __init__(self):
        self.L1 = 0
        self.L2 = 0
        self.R1 = 0
        self.R2 = 0
        self.A = 0
        self.B = 0
        self.X = 0
        self.Y = 0
        self.Up = 0
        self.Down = 0
        self.Left = 0
        self.Right = 0
        self.F1 = 0
        self.Start = 0
        self.Select = 0
        # 添加按键状态的上一次值，用于检测变化
        self._last_values = {}

    def parse(self, data):
        """
        根据xKeySwitchUnion结构解析按键
        
        Args:
            data: 遥控器数据字节数组
        """
        # 第一个字节 [2] 包含 R1, L1, start, select, R2, L2, F1, F2
        self.R1 = (data[2] >> 0) & 1
        self.L1 = (data[2] >> 1) & 1
        self.Start = (data[2] >> 2) & 1
        self.Select = (data[2] >> 3) & 1
        self.R2 = (data[2] >> 4) & 1
        self.L2 = (data[2] >> 5) & 1
        self.F1 = (data[2] >> 6) & 1
        # F2 = (data[2] >> 7) & 1
        
        # 第二个字节 [3] 包含 A, B, X, Y, up, right, down, left
        self.A = (data[3] >> 0) & 1
        self.B = (data[3] >> 1) & 1
        self.X = (data[3] >> 2) & 1
        self.Y = (data[3] >> 3) & 1
        self.Up = (data[3] >> 4) & 1
        self.Right = (data[3] >> 5) & 1
        self.Down = (data[3] >> 6) & 1
        self.Left = (data[3] >> 7) & 1
    
    def is_pressed_once(self, key_name):
        """
        检查按键是否是刚刚按下的（按键边缘检测）
        
        Args:
            key_name: 按键名称
            
        Returns:
            bool: 如果按键是刚刚按下的返回True，否则返回False
        """
        current_value = getattr(self, key_name, 0)
        last_value = self._last_values.get(key_name, 0)
        self._last_values[key_name] = current_value
        return current_value and not last_value
    
    def get_combo_once(self, key1, key2):
        """
        检查组合键是否是刚刚按下的
        
        Args:
            key1: 第一个按键名称
            key2: 第二个按键名称
            
        Returns:
            bool: 如果组合键是刚刚按下的返回True，否则返回False
        """
        current_pressed = getattr(self, key1, 0) and getattr(self, key2, 0)
        last_pressed = self._last_values.get(f"{key1}+{key2}", False)
        self._last_values[f"{key1}+{key2}"] = current_pressed
        return current_pressed and not last_pressed
