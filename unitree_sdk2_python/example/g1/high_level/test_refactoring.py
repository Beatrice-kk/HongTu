#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
测试重构后的模块是否正常工作
"""

import sys
import os

# Add current directory to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def test_imports():
    """Test if all modules can be imported correctly"""
    print("Testing module imports...")
    
    try:
        from g1_joint_index import G1JointIndex
        print("✅ g1_joint_index 导入成功")
        
        # 测试关节索引
        assert hasattr(G1JointIndex, 'WaistYaw')
        assert hasattr(G1JointIndex, 'LeftShoulderPitch')
        assert hasattr(G1JointIndex, 'RightShoulderPitch')
        print("✅ G1JointIndex 常量定义正确")
        
    except Exception as e:
        print(f"❌ g1_joint_index 导入失败: {e}")
        return False
    
    try:
        from remote_controller import RemoteController
        print("✅ remote_controller 导入成功")
        
        # 测试遥控器类
        remote = RemoteController()
        assert hasattr(remote, 'parse')
        assert hasattr(remote, 'is_pressed_once')
        assert hasattr(remote, 'get_combo_once')
        print("✅ RemoteController 类定义正确")
        
    except Exception as e:
        print(f"❌ remote_controller 导入失败: {e}")
        return False
    
    try:
        from g1_loco_mode_checker import G1LocoModeChecker
        print("✅ g1_loco_mode_checker 导入成功")
        
        # 测试运控模式检测器类
        checker = G1LocoModeChecker()
        assert hasattr(checker, 'GetFsmId')
        assert hasattr(checker, 'is_in_main_loco_mode')
        print("✅ G1LocoModeChecker 类定义正确")
        
    except Exception as e:
        print(f"❌ g1_loco_mode_checker 导入失败: {e}")
        return False
    
    try:
        from g1_action_player import G1ActionPlayer
        print("✅ g1_action_player 导入成功")
        
        # 测试动作播放器类（不实际初始化，因为需要SDK环境）
        assert hasattr(G1ActionPlayer, '__init__')
        assert hasattr(G1ActionPlayer, 'play_action')
        assert hasattr(G1ActionPlayer, 'stop_play')
        print("✅ G1ActionPlayer 类定义正确")
        
    except Exception as e:
        print(f"❌ g1_action_player 导入失败: {e}")
        return False
    
    try:
        from g1_client_main import main
        print("✅ g1_client_main 导入成功")
        
        # 测试主函数
        assert callable(main)
        print("✅ main 函数定义正确")
        
    except Exception as e:
        print(f"❌ g1_client_main 导入失败: {e}")
        return False
    
    return True

def test_remote_controller():
    """测试遥控器功能"""
    print("\n🧪 测试遥控器功能...")
    
    try:
        from remote_controller import RemoteController
        
        remote = RemoteController()
        
        # 模拟遥控器数据
        test_data = bytearray([0, 0, 0b00000000, 0b00000000])  # 所有按键都未按下
        remote.parse(test_data)
        
        # 测试按键状态
        assert remote.L1 == 0
        assert remote.A == 0
        assert remote.Start == 0
        print("✅ 遥控器按键解析正常")
        
        # 测试按键按下
        test_data[2] = 0b00000010  # L1按下
        test_data[3] = 0b00000001  # A按下
        remote.parse(test_data)
        
        assert remote.L1 == 1
        assert remote.A == 1
        print("✅ 遥控器按键检测正常")
        
        # 测试组合键检测
        combo_pressed = remote.get_combo_once('L1', 'A')
        assert combo_pressed == True
        print("✅ 遥控器组合键检测正常")
        
        return True
        
    except Exception as e:
        print(f"❌ 遥控器测试失败: {e}")
        return False

def main():
    """主测试函数"""
    print("🚀 开始测试重构后的模块...")
    
    # 测试导入
    if not test_imports():
        print("❌ 模块导入测试失败")
        return False
    
    # 测试遥控器
    if not test_remote_controller():
        print("❌ 遥控器功能测试失败")
        return False
    
    print("\n✅ 所有测试通过！重构成功！")
    print("\n📁 重构后的文件结构:")
    print("  ├── g1_joint_index.py          # 关节索引常量")
    print("  ├── remote_controller.py       # 遥控器解析")
    print("  ├── g1_loco_mode_checker.py    # 运控模式检测器")
    print("  ├── g1_action_player.py        # 动作播放器")
    print("  ├── g1_client_main.py          # 主客户端程序")
    print("  └── g1_client_original_backup.py  # 原始文件备份")
    
    print("\n💡 使用方法:")
    print("  python3 g1_client_main.py  # 运行重构后的主程序")
    print("  python3 g1_client_original_backup.py  # 运行原始程序（如果需要）")
    
    return True

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
