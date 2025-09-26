#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
G1机器人主客户端程序 - 重构版本
"""

import sys
import os
import time
import numpy as np
import subprocess

# 导入ROS相关模块（可选）
try:
    import rospy
    from std_srvs.srv import Trigger, TriggerResponse
    from std_msgs.msg import String as RosString
except Exception:
    rospy = None

# 导入SDK模块
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../../"))
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

# 导入自定义模块
from g1_joint_index import G1JointIndex
from remote_controller import RemoteController
from g1_action_player import G1ActionPlayer
from g1_loco_mode_checker import G1LocoModeChecker

def main(return_remote=False):
    """
    主函数
    
    Args:
        return_remote: 是否只返回remote实例
        
    Returns:
        RemoteController: 遥控器实例
    """
    network_interface = "eth0"

    # 获取当前文件所在目录
    current_dir = os.path.dirname(os.path.abspath(__file__))
    action_dir = os.path.join(current_dir, "action")  # 默认动作目录
    
    # 自动识别网卡
    try:
        with open('/proc/net/dev', 'r') as f:
            content = f.read()
            if network_interface not in content:
                print(f"?? 网卡 '{network_interface}' 不存在")
                lines = content.strip().split('\n')[2:]
                candidates = [line.split(':')[0].strip() for line in lines if 'lo' not in line and 'docker' not in line]
                if candidates:
                    network_interface = candidates[0]
                    print(f"? 自动选择网卡: {network_interface}")
                else:
                    network_interface = "lo"
                    print("? 未找到真实网卡，使用 'lo'")
    except Exception as e:
        print(f"?? 网卡检测失败，使用 'lo': {e}")
        network_interface = "lo"

    # 初始化通信
    try:
        ChannelFactoryInitialize(0, network_interface)
        print(f"? 通信初始化成功: {network_interface}")
    except Exception as e:
        print(f"? 通信初始化失败: {e}")
        if not return_remote:
            sys.exit(-1)
        else:
            return None

    # 创建主运控模式检测器并在程序启动前检查主运控模式
    print("? 检查机器人是否处于主运控模式...")
    loco_checker = G1LocoModeChecker(network_interface)
    
    # 检查机器人是否处于主运控模式
    mode_check_count = 0
    is_in_main_mode = loco_checker.is_in_main_loco_mode()
    
    # 只检查最多10次就继续执行（每次间隔30秒）
    while not is_in_main_mode and mode_check_count < 10 and not return_remote:
        mode_check_count += 1
        print(f"? 机器人未处于主运控模式，30秒后再次检查... (检查次数: {mode_check_count})")
        time.sleep(30)  # 固定等待30秒
        is_in_main_mode = loco_checker.is_in_main_loco_mode()
    
    if is_in_main_mode:
        print("? 机器人已处于主运控模式")
    else:
        print("??  程序将继续执行，但某些功能可能受限")
        print("? 可能的原因:")
        print("   ? 机器人未开机或未站立")
        print("   ? 网络连接问题")
        print("   ? 机器人处于调试模式")
        print("   ? 高层运动服务(ai_sport)未运行")
    
    try:
        player = G1ActionPlayer(action_dir)
        print("? 启动状态反馈订阅...")
    except Exception as e:
        print(f"? 初始化动作播放器失败: {e}")
        if not return_remote:
            sys.exit(-1)
        else:
            return None

    # 创建遥控器解析器
    remote = RemoteController()
    
    # 如果只需要返回remote实例，则在此处返回
    if return_remote:
        return remote
    
    # -------------------------------
    # ROS 服务与话题（若ROS可用则启用）
    # -------------------------------
    ros_available = False
    current_dance_direction = {'value': 'A'}
    if rospy is not None and not return_remote:
        try:
            if not rospy.core.is_initialized():
                # 使用唯一节点名，允许与其他节点共存
                rospy.init_node("g1_dance_service", anonymous=True, disable_signals=True)
            ros_available = True
            print("? ROS 节点已初始化 (g1_dance_service)")

            def _direction_cb(msg: RosString):
                try:
                    val = msg.data
                    if val in ['Up','Down','Left','Right','A','B','X','Y']:
                        current_dance_direction['value'] = val
                        rospy.loginfo(f"dance_direction 设置为: {val}")
                    else:
                        rospy.logwarn(f"无效 dance_direction: {val}")
                except Exception as e:
                    rospy.logerr(f"direction 回调错误: {e}")

            def _handle_play_dance(_req):
                resp = TriggerResponse()
                try:
                    direction = current_dance_direction['value']
                    rospy.loginfo(f"收到 play_dance 请求: {direction}")

                    if direction not in player.actions:
                        available = list(player.actions.keys())
                        resp.success = False
                        resp.message = f"Dance '{direction}' 不存在，可用: {available}"
                        return resp

                    # 若正在播放，先停止
                    if player.state != "stopped":
                        player.stop_play()
                        wait_t0 = time.time()
                        while player.state != "stopped" and (time.time()-wait_t0) < 5.0:
                            time.sleep(0.1)

                    ok = player.play_action(direction, speed=1.0)
                    if not ok:
                        resp.success = False
                        resp.message = f"启动失败: {direction}"
                        return resp

                    # 等待完成（最多120秒）
                    t0 = time.time()
                    while player.state != "stopped" and (time.time()-t0) < 120.0:
                        time.sleep(0.1)

                    if player.state == "stopped":
                        resp.success = True
                        resp.message = f"舞蹈完成: {direction}"
                    else:
                        resp.success = False
                        resp.message = f"舞蹈超时: {direction}"
                except Exception as e:
                    rospy.logerr(f"处理 play_dance 出错: {e}")
                    resp.success = False
                    resp.message = str(e)
                return resp

            rospy.Subscriber("dance_direction", RosString, _direction_cb, queue_size=10)
            rospy.Service("play_dance", Trigger, _handle_play_dance)
            print("? ROS 服务已提供: play_dance，订阅: dance_direction")
        except Exception as e:
            print(f"?? ROS 初始化失败（忽略）：{e}")

    # 创建语音识别订阅者
    try:
        audio_subscriber = ChannelSubscriber("rt/audio_msg", String_)
        
        # 初始化语音识别订阅
        def create_audio_handler(player_instance):
            def handler(msg: String_):
                # 只有在语音控制启用时才处理音频指令
                if player_instance.voice_control_enabled:
                    player_instance.audio_processor.process_audio_message(msg, player_instance.handle_audio_command)
                else:
                    print("? 语音控制已禁用，忽略语音指令")
            return handler
        
        audio_subscriber.Init(create_audio_handler(player), 10)
        player.audio_subscriber = audio_subscriber
    except Exception as e:
        print(f"??  初始化语音识别订阅失败: {e}")
        player.voice_control_enabled = False  # 禁用语音控制
    
    # 等待获取初始位置反馈的标志
    state_flags = {'initial_pose_received': False, 'initialization_done': False}

    def lowstate_callback(msg):
        try:
            # 限制回调函数的处理频率，避免过度消耗CPU
            current_time = time.time()
            if not hasattr(lowstate_callback, '_last_call_time'):
                lowstate_callback._last_call_time = 0
            
            # 限制回调处理频率为50ms一次，平衡响应速度和CPU使用率
            if current_time - lowstate_callback._last_call_time < 0.05:
                return
            
            lowstate_callback._last_call_time = current_time
            
            motor_states = msg.motor_state
            q_feedback = np.zeros(15, dtype=np.float32)
            q_feedback[0] = motor_states[G1JointIndex.WaistYaw].q
            q_feedback[1] = motor_states[G1JointIndex.LeftShoulderPitch].q
            q_feedback[2] = motor_states[G1JointIndex.LeftShoulderRoll].q
            q_feedback[3] = motor_states[G1JointIndex.LeftShoulderYaw].q
            q_feedback[4] = motor_states[G1JointIndex.LeftElbow].q
            q_feedback[5] = motor_states[G1JointIndex.LeftWristRoll].q
            q_feedback[6] = motor_states[G1JointIndex.LeftWristPitch].q
            q_feedback[7] = motor_states[G1JointIndex.LeftWristYaw].q
            q_feedback[8] = motor_states[G1JointIndex.RightShoulderPitch].q
            q_feedback[9] = motor_states[G1JointIndex.RightShoulderRoll].q
            q_feedback[10] = motor_states[G1JointIndex.RightShoulderYaw].q
            q_feedback[11] = motor_states[G1JointIndex.RightElbow].q
            q_feedback[12] = motor_states[G1JointIndex.RightWristRoll].q
            q_feedback[13] = motor_states[G1JointIndex.RightWristPitch].q
            q_feedback[14] = motor_states[G1JointIndex.RightWristYaw].q
            
            # 更新当前姿态
            player.current_pose = q_feedback
            
            # 标记已接收到初始位置反馈
            if not state_flags['initial_pose_received']:
                state_flags['initial_pose_received'] = True
                print("? 接收到初始位置反馈")
            
            # 解析遥控器数据
            remote.parse(msg.wireless_remote)
            
            # 检查机器人是否处于主运控模式（降低检查频率）
            current_time = time.time()
            if not hasattr(lowstate_callback, '_last_mode_check_time'):
                lowstate_callback._last_mode_check_time = 0
            
            # 每2秒检查一次模式状态，避免频繁网络请求
            if current_time - lowstate_callback._last_mode_check_time > 2.0:
                player.in_main_loco_mode = loco_checker.is_in_main_loco_mode()
                lowstate_callback._last_mode_check_time = current_time
            
            # 功能激活检查
            if remote.get_combo_once('F1', 'Start'):
                if player.in_main_loco_mode:
                    player.function_activated = True
                    print("? 功能已激活")
                    if hasattr(player, 'audio_processor') and hasattr(player.audio_processor, 'audio_client'):
                        player.audio_processor.audio_client.TtsMaker("功能已激活", 0)
                else:
                    print("? 机器人未处于主运控模式，无法激活功能")
            
            if remote.get_combo_once('F1', 'Select'):
                player.function_activated = False
                print("? 功能已取消激活")
                if hasattr(player, 'audio_processor') and hasattr(player.audio_processor, 'audio_client'):
                    player.audio_processor.audio_client.TtsMaker("功能已取消激活", 0)
            
            # 语音控制开关
            if remote.get_combo_once('F1', 'L2'):
                player.voice_control_enabled = not player.voice_control_enabled
                status = "启用" if player.voice_control_enabled else "禁用"
                print(f"? 语音控制已{status}")
                if hasattr(player, 'audio_processor') and hasattr(player.audio_processor, 'audio_client'):
                    player.audio_processor.audio_client.TtsMaker(f"语音控制已{status}", 0)
            
            # 只有在功能激活时才处理动作控制
            if player.function_activated:
                # 动作控制
                if remote.get_combo_once('L1', 'Up'):
                    print("? 检测到 L1 + Up，尝试播放向上动作")
                    player.play_action('Up', speed=1.0)
                elif remote.get_combo_once('L1', 'Down'):
                    print("? 检测到 L1 + Down，尝试播放向下动作")
                    player.play_action('Down', speed=1.0)
                elif remote.get_combo_once('L1', 'Left'):
                    print("? 检测到 L1 + Left，尝试播放向左动作")
                    player.play_action('Left', speed=1.0)
                elif remote.get_combo_once('L1', 'Right'):
                    print("? 检测到 L1 + Right，尝试播放向右动作")
                    player.play_action('Right', speed=1.0)
                elif remote.get_combo_once('L1', 'A'):
                    print("? 检测到 L1 + A，尝试播放A动作")
                    player.play_action('A', speed=1.0)
                elif remote.get_combo_once('L1', 'B'):
                    print("? 检测到 L1 + B，尝试播放B动作")
                    player.play_action('B', speed=1.0)
                elif remote.get_combo_once('L1', 'X'):
                    print("? 检测到 L1 + X，尝试播放X动作")
                    player.play_action('X', speed=1.0)
                elif remote.get_combo_once('L1', 'Y'):
                    print("? 检测到 L1 + Y，尝试播放Y动作")
                    player.play_action('Y', speed=1.0)
                
                # 停止动作
                if remote.get_combo_once('L1', 'F1'):
                    print("? 检测到 L1 + F1，停止动作并回到初始姿态")
                    player.stop_play()
                
                # Start + L2: 关闭所有导航建图相关程序
                if remote.get_combo_once('Start', 'L2'):
                    print("? 检测到 Start + L2，关闭所有导航建图相关程序")
                    try:
                        player._shutdown_navigation_systems()
                        if hasattr(player, 'audio_processor') and hasattr(player.audio_processor, 'audio_client'):
                            player.audio_processor.audio_client.TtsMaker("已关闭所有导航程序", 0)
                    except Exception as e:
                        print(f"[shutdown] 关闭失败: {e}")
                
                # Start + L1: 启动 fastlio 导航
                if remote.get_combo_once('Start', 'L1'):
                    print("? 检测到 Start + L1，启动 fastlio 导航")
                    try:
                        player._start_fastlio_navigation_with_monitoring()
                    except Exception as e:
                        print(f"[fastlio] 触发失败: {e}")
                
                # Start + Up: 导航启动≥10秒后触发 simplified_nav_dance.py -a
                if remote.get_combo_once('Start', 'Up'):
                    if player._can_trigger_after_nav():
                        print("? 检测到 Start + Up，触发导航后动作A")
                        try:
                            subprocess.Popen([
                                "bash", "-lc",
                                "python3 /home/unitree/HongTu/PythonProject/point_nav/simplified_nav_dance.py -a"
                            ])
                            if hasattr(player, 'audio_processor') and hasattr(player.audio_processor, 'audio_client'):
                                player.audio_processor.audio_client.TtsMaker("启动动作A", 0)
                        except Exception as e:
                            print(f"[nav_dance] 启动失败: {e}")
                    else:
                        print("? 导航启动时间不足10秒，请稍后再试")
                
                # Start + Down: 导航启动≥10秒后触发 simplified_nav_dance.py -b
                if remote.get_combo_once('Start', 'Down'):
                    if player._can_trigger_after_nav():
                        print("? 检测到 Start + Down，触发导航后动作B")
                        try:
                            subprocess.Popen([
                                "bash", "-lc",
                                "python3 /home/unitree/HongTu/PythonProject/point_nav/simplified_nav_dance.py -b"
                            ])
                            if hasattr(player, 'audio_processor') and hasattr(player.audio_processor, 'audio_client'):
                                player.audio_processor.audio_client.TtsMaker("启动动作B", 0)
                        except Exception as e:
                            print(f"[nav_dance] 启动失败: {e}")
                    else:
                        print("? 导航启动时间不足10秒，请稍后再试")
                
                # Start + Left: 导航启动≥10秒后触发 simplified_nav_dance.py -x
                if remote.get_combo_once('Start', 'Left'):
                    if player._can_trigger_after_nav():
                        print("? 检测到 Start + Left，触发导航后动作X")
                        try:
                            subprocess.Popen([
                                "bash", "-lc",
                                "python3 /home/unitree/HongTu/PythonProject/point_nav/simplified_nav_dance.py -x"
                            ])
                            if hasattr(player, 'audio_processor') and hasattr(player.audio_processor, 'audio_client'):
                                player.audio_processor.audio_client.TtsMaker("启动动作X", 0)
                        except Exception as e:
                            print(f"[nav_dance] 启动失败: {e}")
                    else:
                        print("? 导航启动时间不足10秒，请稍后再试")
                
                # Start + Right: 导航启动≥10秒后触发 simplified_nav_dance.py -y
                if remote.get_combo_once('Start', 'Right'):
                    if player._can_trigger_after_nav():
                        print("? 检测到 Start + Right，触发导航后动作Y")
                        try:
                            subprocess.Popen([
                                "bash", "-lc",
                                "python3 /home/unitree/HongTu/PythonProject/point_nav/simplified_nav_dance.py -y"
                            ])
                            if hasattr(player, 'audio_processor') and hasattr(player.audio_processor, 'audio_client'):
                                player.audio_processor.audio_client.TtsMaker("启动动作Y", 0)
                        except Exception as e:
                            print(f"[nav_dance] 启动失败: {e}")
                    else:
                        print("? 导航启动时间不足10秒，请稍后再试")
                
                # Start + R1: 导航启动≥10秒后触发 simplified_nav_dance.py -Up
                if remote.get_combo_once('Start', 'R1'):
                    if player._can_trigger_after_nav():
                        print("? 检测到 Start + R1，触发导航后动作Up")
                        try:
                            subprocess.Popen([
                                "bash", "-lc",
                                "python3 /home/unitree/HongTu/PythonProject/point_nav/simplified_nav_dance.py -Up"
                            ])
                            if hasattr(player, 'audio_processor') and hasattr(player.audio_processor, 'audio_client'):
                                player.audio_processor.audio_client.TtsMaker("启动动作Up", 0)
                        except Exception as e:
                            print(f"[nav_dance] 启动失败: {e}")
                    else:
                        print("? 导航启动时间不足10秒，请稍后再试")
                
                # Start + R2: 导航启动≥10秒后触发 simplified_nav_dance.py -Down
                if remote.get_combo_once('Start', 'R2'):
                    if player._can_trigger_after_nav():
                        print("? 检测到 Start + R2，触发导航后动作Down")
                        try:
                            subprocess.Popen([
                                "bash", "-lc",
                                "python3 /home/unitree/HongTu/PythonProject/point_nav/simplified_nav_dance.py -Down"
                            ])
                            if hasattr(player, 'audio_processor') and hasattr(player.audio_processor, 'audio_client'):
                                player.audio_processor.audio_client.TtsMaker("启动动作Down", 0)
                        except Exception as e:
                            print(f"[nav_dance] 启动失败: {e}")
                    else:
                        print("? 导航启动时间不足10秒，请稍后再试")
                
                # 更新动作播放器
                player.update()
            else:
                # 功能未激活时，让机器人可以正常响应遥控器控制
                if player.state not in ["stopped", "move_to_initial"]:
                    print("??  功能未激活，正在停止当前动作...")
                    player.stop_play()
                
                # 使用低频更新以降低CPU使用率，但在stopped状态下不调用
                if player.state != "stopped":
                    player.update()
                time.sleep(0.1)  # 100ms延迟
        except Exception as e:
            print(f"? 回调处理失败: {e}")
            pass  # 添加pass语句确保语法正确
    
    subscriber = ChannelSubscriber("rt/lowstate", LowState_)
    subscriber.Init(lowstate_callback, 10)
    
    # 等待接收初始关节位置反馈
    print("? 等待接收初始关节位置反馈...")
    wait_start = time.time()
    # 等待最多5秒
    while not state_flags['initial_pose_received'] and (time.time() - wait_start) < 5.0:
        time.sleep(0.1)
    
    # 即使没有收到反馈也继续执行，使用默认零位姿态
    if not state_flags['initial_pose_received']:
        print("?? 超时未收到初始位置反馈，使用默认零位姿态继续执行")
        if player.current_pose is None:
            player.current_pose = np.zeros(15, dtype=np.float32)
        state_flags['initial_pose_received'] = True
        state_flags['initialization_done'] = True  # 标记为已完成初始化
    else:
        print("? 成功接收到初始位置反馈")
    
    # 确保player已经接收到了当前位置反馈
    if player.current_pose is None:
        player.current_pose = np.zeros(15, dtype=np.float32)
        print("? 使用默认零位姿态")
    
    # 程序启动后立即执行初始化到零位
    print("? 开始初始化流程...")
    player.init_to_zero_position()
    print("? 初始化流程完成")
    
    print("? 程序初始化完成")
    print("? 功能当前未激活，请在机器人处于主运控模式时按 F1 + Start 激活功能")
    print("??  程序将持续检测机器人模式状态")
    print("? 操作说明:")
    print("  ┌──────────────┬────────────────────────────┐")
    print("  │   按键组合   │        功能说明          │")
    print("  ├──────────────┼────────────────────────────┤")
    print("  │  L1 + Up     │  播放向上动作             │")
    print("  │  L1 + Down   │  播放向下动作             │")
    print("  │  L1 + Left   │  播放向左动作             │")
    print("  │  L1 + Right  │  播放向右动作             │")
    print("  │  L1 + A      │  播放A动作                │")
    print("  │  L1 + B      │  播放B动作                │")
    print("  │  L1 + X      │  播放X动作                │")
    print("  │  L1 + Y      │  播放Y动作                │")
    print("  │  L1 + F1     │  取消播放并回到初始姿态    │")
    print("  │  F1 + Start  │  激活功能                 │")
    print("  │  F1 + Select │  取消激活功能             │")
    print("  │  F1 + L2     │  开启/关闭语音控制        │")
    print("  │  Start + L1  │  启动 fastlio 导航        │")
    print("  │  Start + Up  │  触发导航后动作           │")
    print("  └──────────────┴────────────────────────────┘")
    print("??  语音指令:")
    print("  ┌──────────────┬────────────────────────────┐")
    print("  │   指令内容   │        功能说明          │")
    print("  ├──────────────┼────────────────────────────┤")
    print("  │ 小G / 你好   │  唤醒机器人              │")
    print("  │ 播放 / 开始  │  开始播放动作             │")
    print("  │ 停止 / 结束  │  停止播放动作             │")
    print("  │    循环      │  切换循环播放模式         │")
    print("  └──────────────┴────────────────────────────┘")
    print("??  语音控制当前状态: 禁用" if not player.voice_control_enabled else "??  语音控制当前状态: 启用")
    
    # 显示加载的动作
    print("? 已加载的动作:")
    for direction, action in player.actions.items():
        print(f"  {direction}: {action['name']}")

    try:
        while True:
            time.sleep(0.1)  # 减少CPU占用
    except KeyboardInterrupt:
        print("\n? 收到中断信号，准备退出")
    except Exception as e:
        print(f"\n? 程序运行出错: {e}")
        import traceback
        traceback.print_exc()
        
    # 程序退出前确保关闭所有资源
    try:
        if player and hasattr(player, 'audio_processor'):
            # 添加防重复机制
            if not hasattr(player, '_last_exit_tts_time'):
                player._last_exit_tts_time = 0
            current_time = time.time()
            if current_time - player._last_exit_tts_time > 3.0:  # 至少间隔3秒
                player.audio_processor.audio_client.TtsMaker("程序即将退出", 0)
                player._last_exit_tts_time = current_time
    except Exception as e:
        print(f"? 退出提示失败: {e}")

    print("\n? 程序退出")
    
    # 返回remote实例供其他程序使用
    return remote


if __name__ == "__main__":
    main()
