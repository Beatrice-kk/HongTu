#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import math
from collections import deque
from typing import Deque
from std_srvs.srv import SetBool, SetBoolResponse

# 假设 unitree_sdk2py 已经正确安装
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


class CmdVelController:
    """
    Controller for Unitree G1 robot with dynamic rotation control
    """

    def __init__(self, network_interface: str):
        rospy.loginfo("Initializing Unitree Controller with Oscillation Detection...")

        # --- 参数初始化 ---
        self._load_ros_params()

        rospy.loginfo(
            f"Controller params: \n"
            f"  - Oscillation Detection: buffer_size={self.WZ_BUFFER_SIZE}, "
            f"magnitude_thresh={self.OSCILLATION_MAGNITUDE_THRESHOLD:.2f} rad/s, "
            f"cooldown={self.COOLDOWN_DURATION:.1f}s\n"
            f"  - Velocity Thresholds: linear={self.VEL_LINEAR_THRESHOLD:.3f} m/s, "
            f"angular={self.VEL_ANGULAR_THRESHOLD:.3f} rad/s\n"
            f"  - Initial Rotation Control: flag_rotate={self.flag_rotate} "
            f"(0=avoid rotation, 1=normal rotation)"
        )

        # --- 状态变量 ---
        self.path_is_valid: bool = False
        self.last_valid_cmd_time: rospy.Time = rospy.Time.now()
        self.wz_buffer: Deque[float] = deque(maxlen=self.WZ_BUFFER_SIZE)
        self.cooldown_active: bool = False
        self.cooldown_end_time: rospy.Time = rospy.Time.now()

        # --- SDK初始化 ---
        rospy.loginfo("Initializing Unitree LocoClient...")
        ChannelFactoryInitialize(0, network_interface)
        self.sport_client = LocoClient()
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()

        # 选择步态
        self.sport_client.WalkMotion()

        # --- ROS接口 ---
        # 核心控制接口
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback, queue_size=1)
        rospy.Subscriber(
            "/move_base/GlobalPlanner/plan", Path, self.path_callback, queue_size=1
        )
        self.stop_timer = rospy.Timer(rospy.Duration(0.5), self.check_cmd_timeout)

        # 创建旋转控制服务
        self.rotation_service = rospy.Service(
            "~set_rotation_enabled", SetBool, self.set_rotation_enabled
        )
        rospy.loginfo(
            "Rotation control service is now available at: "
            + rospy.get_name()
            + "/set_rotation_enabled"
        )

    def _load_ros_params(self):
        """从 ROS 参数服务器加载所有参数。"""
        # 震荡检测参数
        self.WZ_BUFFER_SIZE = rospy.get_param("~wz_buffer_size", 6)
        self.OSCILLATION_MAGNITUDE_THRESHOLD = rospy.get_param(
            "~oscillation_magnitude_threshold", 0.07
        )
        self.COOLDOWN_DURATION = rospy.get_param("~cooldown_duration", 3.0)
        # 速度阈值参数
        self.VEL_LINEAR_THRESHOLD = rospy.get_param("~vel_linear_threshold", 0.03)
        self.VEL_ANGULAR_THRESHOLD = rospy.get_param("~vel_angular_threshold", 0.05)
        # 旋转控制参数 - 使用实例变量以便动态修改
        self.flag_rotate = rospy.get_param("~flag_rotate", 1)  # 默认值为1，允许正常旋转

    def set_rotation_enabled(self, req):
        """ROS服务回调，用于设置flag_rotate参数"""
        old_value = self.flag_rotate
        self.flag_rotate = 1 if req.data else 0
        rospy.loginfo(
            f"Rotation control changed from {old_value} to {self.flag_rotate}"
        )
        return SetBoolResponse(
            success=True, message=f"Rotation control set to {self.flag_rotate}"
        )

    def path_callback(self, msg: Path):
        """根据全局路径是否存在来更新路径有效性标志。"""
        new_path_is_valid = len(msg.poses) > 0
        if new_path_is_valid and not self.path_is_valid:
            rospy.loginfo("Global path received, robot can start moving.")
            # 收到新路径时，重置冷却状态，让机器人可以立即响应新规划
            self._reset_cooldown()
        elif not new_path_is_valid and self.path_is_valid:
            rospy.logwarn("Global path became empty, forcing robot to stop.")
            self.force_stop()

        self.path_is_valid = new_path_is_valid

    def cmd_vel_callback(self, msg: Twist):
        """处理 /cmd_vel 消息的核心回调函数。"""
        # 1. 检查是否处于冷却状态
        if self._is_in_cooldown():
            rospy.loginfo_throttle(
                1.0, "In cooldown after oscillation, ignoring cmd_vel."
            )
            return

        # 2. 检查路径是否有效
        if not self.path_is_valid:
            rospy.logwarn_throttle(
                1.0, "Global path not received or empty. Ignoring cmd_vel."
            )
            self.force_stop()
            return

        vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z

        # 处理旋转控制标志
        original_wz = wz
        if self.flag_rotate == 0 and abs(wz) > 0.01:
            # 当禁止旋转且有旋转指令时
            if abs(wz) > 0.5:  # 如果是大幅度旋转（约30度/秒）
                rospy.logwarn_throttle(
                    1.0, f"Rotation command {wz:.2f} rad/s ignored due to flag_rotate=0"
                )
                wz = 0.0

                # 如果需要转向，优先使用侧向移动代替旋转
                if vx > 0:  # 如果机器人正在前进
                    # 根据原始旋转方向决定侧向移动方向
                    if original_wz > 0:  # 原本要向左转
                        vy = max(vy, 0.2)  # 向左侧移动
                    else:  # 原本要向右转
                        vy = min(vy, -0.2)  # 向右侧移动
                elif vx < 0:  # 如果是倒退，则侧向移动方向相反
                    if original_wz > 0:  # 原本要向左转
                        vy = min(vy, -0.2)  # 向右侧移动
                    else:  # 原本要向右转
                        vy = max(vy, 0.2)  # 向左侧移动
            else:
                # 小幅度旋转可以保留，但减小幅度
                wz *= 0.3
                rospy.loginfo_throttle(
                    2.0, f"Reduced rotation from {original_wz:.2f} to {wz:.2f} rad/s"
                )

        self.wz_buffer.append(wz)

        # 3. 检查是否发生震荡
        if self._check_for_oscillation():
            rospy.logwarn("Oscillation detected! Activating cooldown.")
            self._start_cooldown()
            self.force_stop()
            return

        # 4. 忽略过小的指令
        linear_vel_magnitude = math.hypot(vx, vy)
        if (
            linear_vel_magnitude < self.VEL_LINEAR_THRESHOLD
            and abs(wz) < self.VEL_ANGULAR_THRESHOLD
        ):
            rospy.loginfo_throttle(
                5.0,
                f"Ignoring small cmd_vel (v={linear_vel_magnitude:.3f}, w={wz:.3f}) below threshold.",
            )
            return

        # 5. 执行有效指令
        self.last_valid_cmd_time = rospy.Time.now()
        rospy.loginfo_throttle(
            1.0, f"Executing cmd_vel: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}"
        )
        try:
            self.sport_client.Move(vx, vy, wz)
        except Exception as e:
            rospy.logerr(f"Failed to send Move command: {e}")

    def check_cmd_timeout(self, event):
        """定时检查指令是否超时，确保安全。"""
        is_timeout = (rospy.Time.now() - self.last_valid_cmd_time) > rospy.Duration(0.4)
        if self.path_is_valid and not self.cooldown_active and is_timeout:
            rospy.logwarn("No valid cmd_vel for a while. Forcing robot to stop.")
            self.force_stop()

    def force_stop(self):
        """发送明确的停止指令并清空历史角速度，以防影响下一次判断。"""
        rospy.loginfo("Executing FORCE STOP.")
        try:
            self.sport_client.StopMove()
        except Exception as e:
            rospy.logerr(f"Failed to send stop command: {e}")
        self.wz_buffer.clear()

    def _check_for_oscillation(self) -> bool:
        """检查 wz_buffer 中的数据是否构成震荡模式。"""
        if len(self.wz_buffer) < self.WZ_BUFFER_SIZE:
            return False

        # 幅度检查：所有值的绝对值都必须小于阈值
        if any(abs(wz) > self.OSCILLATION_MAGNITUDE_THRESHOLD for wz in self.wz_buffer):
            return False

        # 符号检查：必须同时存在正值和负值（忽略接近零的值）
        has_positive = any(wz > 0.01 for wz in self.wz_buffer)
        has_negative = any(wz < -0.01 for wz in self.wz_buffer)

        if has_positive and has_negative:
            rospy.logwarn(
                f"OSCILLATION PATTERN DETECTED! Recent wz: {[f'{w:.3f}' for w in self.wz_buffer]}"
            )
            return True

        return False

    def _is_in_cooldown(self) -> bool:
        """检查当前是否处于冷却状态。"""
        if self.cooldown_active:
            if rospy.Time.now() < self.cooldown_end_time:
                return True
            else:
                rospy.loginfo("Cooldown finished. Resuming normal operation.")
                self._reset_cooldown()
        return False

    def _start_cooldown(self):
        """启动冷却状态。"""
        self.cooldown_active = True
        self.cooldown_end_time = rospy.Time.now() + rospy.Duration(
            self.COOLDOWN_DURATION
        )

    def _reset_cooldown(self):
        """重置冷却状态。"""
        if self.cooldown_active:
            rospy.loginfo("Resetting cooldown state.")
        self.cooldown_active = False
        self.wz_buffer.clear()

    def shutdown(self):
        """节点关闭时执行的操作。"""
        rospy.loginfo("Shutting down controller. Forcing robot to stop.")
        self.force_stop()


if __name__ == "__main__":
    rospy.init_node("unitree_cmd_vel_controller", anonymous=False)

    try:
        network_interface = rospy.get_param("~network_interface", "eth0")
        rospy.logwarn("Make sure the robot is in a safe environment before running!")

        controller = CmdVelController(network_interface)
        rospy.on_shutdown(controller.shutdown)  # 注册关闭时的回调函数
        rospy.spin()

    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.loginfo("Controller shutdown requested.")
    except Exception as e:
        rospy.logfatal(f"An unhandled exception occurred during initialization: {e}")
