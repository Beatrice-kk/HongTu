#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import sys
import math
from collections import deque # 引入双端队列

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

class CmdVelController:
    def __init__(self, network_interface):
        rospy.loginfo("Initializing Unitree Controller with Oscillation Detection...")

        # --- 震荡检测相关参数 ---
        # 存储最近多少个 wz 指令用于分析
        self.WZ_BUFFER_SIZE = rospy.get_param("~wz_buffer_size", 6)
        # 如果 wz 绝对值都小于这个值，才被认为是震荡候选
        self.OSCILLATION_MAGNITUDE_THRESHOLD = rospy.get_param("~oscillation_magnitude_threshold", 0.07) # rad/s
        # 检测到震荡后，锁定忽略 cmd_vel 的时间
        self.COOLDOWN_DURATION = rospy.get_param("~cooldown_duration", 3.0) # seconds

        rospy.loginfo(f"Oscillation detection params: buffer_size={self.WZ_BUFFER_SIZE}, magnitude_thresh={self.OSCILLATION_MAGNITUDE_THRESHOLD}, cooldown={self.COOLDOWN_DURATION}s")

        # --- 基础参数 ---
        self.VEL_LINEAR_THRESHOLD = rospy.get_param("~vel_linear_threshold", 0.03)
        self.VEL_ANGULAR_THRESHOLD = rospy.get_param("~vel_angular_threshold", 0.05)

        # --- 状态变量 ---
        self.path_is_valid = False
        self.last_valid_cmd_time = rospy.Time.now()
        # 使用 deque 来自动管理一个固定大小的队列
        self.wz_buffer = deque(maxlen=self.WZ_BUFFER_SIZE)
        self.cooldown_active = False
        self.cooldown_end_time = rospy.Time.now()

        # --- Unitree SDK 初始化 ---
        rospy.loginfo("Initializing Unitree LocoClient...")
        ChannelFactoryInitialize(0, network_interface)
        self.sport_client = LocoClient()
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()

        # --- ROS 订阅与定时器 ---
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback, queue_size=1)
        rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.path_callback, queue_size=1)
        self.stop_timer = rospy.Timer(rospy.Duration(0.5), self.check_cmd_timeout)

    def path_callback(self, msg: Path):
        """根据全局路径是否存在来更新路径有效性标志"""
        if len(msg.poses) > 0:
            if not self.path_is_valid:
                rospy.loginfo("Global path received, robot can start moving.")
                # 当收到新路径时，如果之前在冷却状态，则解除
                if self.cooldown_active:
                    rospy.loginfo("New path received, resetting cooldown state.")
                    self.cooldown_active = False
                    self.wz_buffer.clear()
            self.path_is_valid = True
        else:
            if self.path_is_valid:
                rospy.logwarn("Global path is empty, robot will stop.")
                self.force_stop()
            self.path_is_valid = False

    def check_for_oscillation(self):
        """检查 wz_buffer 中的数据是否构成震荡模式"""
        # 1. 确保有足够的数据
        if len(self.wz_buffer) < self.WZ_BUFFER_SIZE:
            return False

        # 2. 检查所有值的幅度是否都在阈值内
        if any(abs(wz) > self.OSCILLATION_MAGNITUDE_THRESHOLD for wz in self.wz_buffer):
            return False

        # 3. 检查是否同时存在正值和负值
        has_positive = any(wz > 0.01 for wz in self.wz_buffer) # 使用一个小的死区避免零
        has_negative = any(wz < -0.01 for wz in self.wz_buffer)

        if has_positive and has_negative:
            rospy.logwarn(f"OSCILLATION DETECTED! Recent wz: {[f'{w:.2f}' for w in self.wz_buffer]}")
            return True
        
        return False

    def cmd_vel_callback(self, msg: Twist):
        # 如果正在冷却期，检查是否结束
        if self.cooldown_active:
            if rospy.Time.now() < self.cooldown_end_time:
                rospy.loginfo("In cooldown after oscillation detection, ignoring cmd_vel.")
                return
            else:
                rospy.loginfo("Cooldown finished. Resuming normal operation.")
                self.cooldown_active = False
                self.wz_buffer.clear()

        # 如果路径无效，则直接停止并返回
        if not self.path_is_valid:
            rospy.logwarn("Global path not received or empty. Ignoring cmd_vel.")
            self.force_stop()
            return
            
        vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z
        self.wz_buffer.append(wz)

        if self.check_for_oscillation():
            rospy.logwarn("Activating cooldown due to oscillation.")
            self.force_stop()
            self.cooldown_active = True
            self.cooldown_end_time = rospy.Time.now() + rospy.Duration(self.COOLDOWN_DURATION)
            return

        linear_vel_magnitude = math.sqrt(vx**2 + vy**2)
        if linear_vel_magnitude < self.VEL_LINEAR_THRESHOLD and abs(wz) < self.VEL_ANGULAR_THRESHOLD:
            rospy.loginfo(f"Ignoring small cmd_vel (vx={vx:.3f}, wz={wz:.3f}) below threshold.")
            return

        self.last_valid_cmd_time = rospy.Time.now()
        rospy.loginfo(f"Executing cmd_vel: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}")
        try:
            self.sport_client.Move(vx, vy, wz)
        except Exception as e:
            rospy.logerr(f"Failed to send Move command: {e}")

    def check_cmd_timeout(self, event):
        """定时器回调，用于检查是否长时间没有收到有效指令"""
        if not self.cooldown_active and self.path_is_valid and \
           (rospy.Time.now() - self.last_valid_cmd_time > rospy.Duration(0.4)):
            rospy.logwarn("No valid cmd_vel for a while. Forcing robot to stop.")
            self.force_stop()

    def force_stop(self):
        """发送明确的停止指令"""
        rospy.loginfo("Executing FORCE STOP.")
        try:
            self.sport_client.Move(0, 0, 0)
        except Exception as e:
            rospy.logerr(f"Failed to send stop command: {e}")
        # 停止后清空历史记录，避免旧数据影响下一次判断
        self.wz_buffer.clear()

if __name__ == "__main__":
    rospy.init_node("unitree_cmd_vel_controller", anonymous=False)
    network_interface = rospy.get_param("~network_interface", "eth0")
    rospy.logwarn("Make sure the robot is in a safe environment before running!")
    
    try:
        controller = CmdVelController(network_interface)
        rospy.spin()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.loginfo("Shutting down controller.")
    except Exception as e:
        rospy.logfatal(f"An unhandled exception occurred: {e}")