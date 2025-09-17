import rospy
import math
import tf.transformations as tft
import actionlib
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import PoseStamped, Pose
from typing import Optional
import sys
import time
import threading

# 将包含g1_client_cwk.py的目录添加到路径中
sys.path.append("/home/unitree/unitree_sdk2_python/example/g1/high_level")

from g1_client_cwk import (
    G1ActionPlayer,
    ChannelFactoryInitialize,
    ChannelSubscriber,
    LowState_
)

class DanceController:
    def __init__(self, network_interface="eth0"):
        # 初始化通信
        try:
            ChannelFactoryInitialize(0, network_interface)
            print(f"✅ 通信初始化成功: {network_interface}")
        except Exception as e:
            print(f"❌ 通信初始化失败: {e}")
            raise e
            
        # 创建动作播放器
        self.action_dir = "/home/unitree/unitree_sdk2_python/example/g1/high_level/action"
        self.player = G1ActionPlayer(self.action_dir)
        
        # 设置状态订阅器
        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self._lowstate_callback, 10)
        
        # 初始化状态
        self.initialized = False
        
        # 等待初始化
        print("⏳ 等待初始化...")
        wait_start = time.time()
        while not self.initialized and (time.time() - wait_start) < 5.0:
            time.sleep(0.1)
        
        if not self.initialized:
            print("⚠️ 等待初始化超时")
        else:
            print("✅ 初始化完成")
    
    def _lowstate_callback(self, msg):
        """处理来自机器人的状态反馈"""
        try:
            # 更新播放器的当前姿态
            motor_states = msg.motor_state
            import numpy as np
            
            if not hasattr(self, 'current_pose'):
                self.current_pose = np.zeros(15, dtype=np.float32)
                
            from g1_client_cwk import G1JointIndex
            
            # 更新腰部位置
            self.current_pose[0] = motor_states[G1JointIndex.WaistYaw].q
            
            # 更新左臂位置
            left_indices = [15,16,17,18,19,20,21]
            for j, idx in enumerate(left_indices):
                self.current_pose[1+j] = motor_states[idx].q
                
            # 更新右臂位置
            right_indices = [22,23,24,25,26,27,28]
            for j, idx in enumerate(right_indices):
                self.current_pose[8+j] = motor_states[idx].q
            
            # 更新播放器的当前姿态
            self.player.current_pose = self.current_pose
            
            # 标记为已初始化
            if not self.initialized:
                self.initialized = True
                
            # 更新播放器状态
            self.player.update()
            
        except Exception as e:
            print(f"❌ 回调错误: {e}")
    
    def play_dance(self, direction, speed=1.0, wait_for_completion=True):
      #   """
      #   播放指定方向的舞蹈
        
      #   参数:
      #       direction: 'Up', 'Down', 'Left', 'Right', 'A', 'B', 'X', 'Y'
      #       speed: 速度倍数（默认：1.0）
      #       wait_for_completion: 是否等待舞蹈完成
            
      #   返回:
      #       如果舞蹈成功启动返回True，否则返回False
      #   """
      #   # 开始舞蹈
      #   result = self.player.play_action(direction, speed)
        
      #   if not result:
      #       print(f"❌ 播放舞蹈失败: {direction}")
      #       return False
            
      #   print(f"▶️ 正在播放舞蹈: {direction}")
        
      #   # 如果请求等待完成
      #   if wait_for_completion:
      #       print("⏳ 等待舞蹈完成...")
      #       while self.player.state != "stopped":
      #           time.sleep(0.1)
      #       print("✅ 舞蹈完成")
            
      #   return True
      pass
    
    def stop_dance(self):
        """停止当前舞蹈"""
        if self.player.state != "stopped":
            self.player.stop_play()
            print("⏹️ 舞蹈已停止")
            return True
        return False
    
    def list_available_dances(self):
        """列出所有可用的舞蹈"""
        dances = []
        for direction, action in self.player.actions.items():
            dances.append({
                "direction": direction,
                "name": action['name'],
                "frames": len(action['data']),
                "duration": len(action['data']) / action['fps']
            })
        return dances


class NavPointSequence:
    def __init__(self, waypoints):
        """
        初始化导航序列处理器。
        :param waypoints: 一个包含(x, y, yaw_deg)元组的列表。
        """
        self.waypoints = waypoints
        self.tried_waypoints = set()  # 存储已经尝试过的航点

        self.dance_index = 4
        self.dance_sequence = ['Up', 'Down', 'Left', 'Right', 'A', 'B', 'X', 'Y']  # 舞蹈序列

        # 初始化 move_base action 客户端
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("正在等待 move_base action 服务器...")
        self.client.wait_for_server()
        rospy.loginfo("move_base action 服务器已连接")

        # 初始化 make_plan 服务客户端
        rospy.loginfo("正在等待 /move_base/make_plan 服务...")
        rospy.wait_for_service('/move_base/make_plan')
        self.make_plan_srv = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        rospy.loginfo("/move_base/make_plan 服务已连接")

        self.current_pose = None
        self.odom_sub = rospy.Subscriber('/slam_odom', Odometry, self.odom_callback, queue_size=1)

        rospy.loginfo("等待里程计数据...")
        while self.current_pose is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("成功接收到里程计数据")

        # 初始化舞蹈控制器
        rospy.loginfo("初始化舞蹈控制器...")
        try:
            self.dance_controller = DanceController()
            self.available_dances = self.dance_controller.list_available_dances()
            rospy.loginfo(f"舞蹈控制器初始化完成，共有 {len(self.available_dances)} 个可用舞蹈")
        except Exception as e:
            rospy.logerr(f"舞蹈控制器初始化失败: {e}")
            self.dance_controller = None

    def odom_callback(self, msg: Odometry):
        """里程计回调函数，更新当前机器人位姿。"""
        self.current_pose = msg.pose.pose

    def is_feasible(self, start_pose: PoseStamped, goal_pose: PoseStamped) -> bool:
        """查看路径是否可行"""
        try:
            resp = self.make_plan_srv(start_pose, goal_pose, 0.2)
            return bool(resp.plan.poses)
        except rospy.ServiceException as e:
            rospy.logerr(f"make_plan 服务调用失败: {e}")
            return False
         
    def build_pose_stamped(self, x: float, y: float, yaw_deg: float) -> PoseStamped:
        """根据x, y, yaw构建PoseStamped消息。"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        q = tft.quaternion_from_euler(0, 0, math.radians(yaw_deg))
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose
    
    def build_move_base_goal(self, pose_stamped: PoseStamped) -> MoveBaseGoal:
        """构建MoveBaseGoal消息"""
        goal = MoveBaseGoal()
        goal.target_pose = pose_stamped
        return goal
    
    def find_nearest_feasible_around(self, orig_wp: tuple, search_radius=0.3, step=0.1, angles=24) -> Optional[tuple]:
        """环形采样可行的航点"""
        x0, y0, yaw = orig_wp
        start_pose = PoseStamped()
        start_pose.header.frame_id = "map"
        start_pose.header.stamp = rospy.Time.now()
        start_pose.pose = self.current_pose

        # 1. 首先检查原始航点是否可行
        goal_pose = self.build_pose_stamped(x0, y0, yaw)
        if self.is_feasible(start_pose, goal_pose):
            rospy.loginfo(f"原始航点 ({x0:.2f}, {y0:.2f}) 可行。")
            return (x0, y0, yaw)

        # 2. 如果不可行，则在周围进行环形采样
        rospy.logwarn(f"原始航点 ({x0:.2f}, {y0:.2f}) 不可达，开始在其附近搜索...")
        for r in np.arange(step, search_radius + step, step):
            for theta in np.linspace(0, 2 * math.pi, angles, endpoint=False):
                x = x0 + r * math.cos(theta)
                y = y0 + r * math.sin(theta)
                goal_pose = self.build_pose_stamped(x, y, yaw)
                if self.is_feasible(start_pose, goal_pose):
                    rospy.loginfo(f"[补偿] 找到新的可行航点: x={x:.2f}, y={y:.2f}, yaw={yaw}")
                    return (x, y, yaw)
        
        rospy.logerr(f"[补偿] 在航点 ({x0:.2f}, {y0:.2f}) 附近 {search_radius}m 范围内未找到可行点。")
        return None

    def get_feasible_nearest_waypoint(self) -> Optional[tuple]:
        """寻找最近的可行航点。"""
        if self.current_pose is None:
            rospy.logwarn("当前位置未知，无法规划路径。")
            return None

        feasible_points = []
        
        # 筛选出还未尝试过的航点
        waypoints_to_check = [wp for wp in self.waypoints if wp not in self.tried_waypoints]
        if not waypoints_to_check:
            rospy.loginfo("所有航点均已尝试。")
            return None

        for wp in waypoints_to_check:
            # 尝试寻找一个可行的航点（可能是原始的或补偿后的）
            adj_wp = self.find_nearest_feasible_around(wp)
            if adj_wp is not None:
                x, y, yaw = adj_wp
                dx = x - self.current_pose.position.x
                dy = y - self.current_pose.position.y
                dist = math.sqrt(dx*dx + dy*dy)
                feasible_points.append(((x, y, yaw), dist))

        if not feasible_points:
            rospy.logwarn("在所有剩余的航点中，没有找到任何一个可行的目标。")
            return None

        # 按距离排序，返回最近的一个
        feasible_points.sort(key=lambda p: p[1])
        return feasible_points[0][0]

    def stabilize_position(self, goal: MoveBaseGoal, duration: rospy.Duration):
        """跳舞的时候保持位置稳定"""
        start_time = rospy.Time.now()
        rate = rospy.Rate(1)  # 以 1 Hz 的频率重新发送目标

        while rospy.Time.now() - start_time < duration and not rospy.is_shutdown():
            goal.target_pose.header.stamp = rospy.Time.now()
            self.client.send_goal(goal)
            rospy.loginfo("    [稳定] 重新发送目标以保持位置...")
            rate.sleep()
        
        # 稳定结束后，取消最后一个目标，让机器人可以自由移动
        self.client.cancel_goal()
        rospy.loginfo("位置稳定结束。")

    def perform_dance(self, goal: MoveBaseGoal):
        """执行舞蹈动作，同时保持位置稳定"""
        if self.dance_controller is None:
            rospy.logwarn("舞蹈控制器未初始化，跳过舞蹈动作")
            return

        # 选择舞蹈动作
        if not self.available_dances:
            rospy.logwarn("没有可用的舞蹈动作")
            return

        dance_direction = self.dance_sequence[self.dance_index % len(self.dance_sequence)]
        self.dance_index += 1

        # 创建一个线程来执行舞蹈，这样不会阻塞导航
        dance_thread = threading.Thread(
            target=self._execute_dance_with_stabilization,
            args=(dance_direction, goal)
        )
        dance_thread.daemon = True  # 设置为守护线程，这样如果主程序退出，舞蹈线程也会退出
        dance_thread.start()

        # 等待舞蹈线程完成
        dance_thread.join()

    def _execute_dance_with_stabilization(self, dance_direction: str, goal: MoveBaseGoal):
        """在一个线程中执行舞蹈并保持位置稳定"""
        try:
            rospy.loginfo(f"🤖 准备舞蹈: {dance_direction}")
            self.stabilize_position(goal, rospy.Duration(1))
            
            dance_thread = threading.Thread(
                target=self.dance_controller.play_dance,
                args=(dance_direction, 1.0, True)  # 以正常速度播放，并等待完成
            )
            dance_thread.start()
            
            # direction: 'Up', 'Down', 'Left', 'Right', 'A', 'B', 'X', 'Y'
            
            dance_time_map = {
               0: 80.0,   # dance_index=0 -> 运行5秒
               1: 80.2,   # dance_index=1 -> 运行8.2秒
               2: 30.5,   # dance_index=2 -> 运行3.5秒
               3: 100.0,  # dance_index=3 -> 运行10秒
            }
            
            
            
            print("舞蹈持续的时间 ")
            print(dance_time_map(self.dance_index))
            print(dance_time_map(self.dance_index))
            print(dance_time_map(self.dance_index))
            
            
            stabilize_thread = threading.Thread(
                target=self.stabilize_position,
                args=(goal, rospy.Duration(dance_time_map.get(self.dance_index,60)))  # 假设舞蹈最多持续10秒
            )
            stabilize_thread.start()
            
            dance_thread.join()
            
            if stabilize_thread.is_alive():
                self.client.cancel_goal()  # 取消稳定目标
            
            rospy.loginfo(f"🎉 舞蹈 {dance_direction} 完成")
            
        except Exception as e:
            rospy.logerr(f"舞蹈执行出错: {e}")
            # 确保停止舞蹈
            try:
                self.dance_controller.stop_dance()
            except:
                pass

    def run_sequence(self):
      #   index = 0

        while not rospy.is_shutdown():
            wp = self.get_feasible_nearest_waypoint()
            if wp is None:
                rospy.loginfo("没有更多可行的航点，任务结束。")
                break

            self.tried_waypoints.add(wp)
            x, y, yaw_deg = wp
            
            # 构建并发送导航目标
            target_pose_stamped = self.build_pose_stamped(x, y, yaw_deg)
            goal = self.build_move_base_goal(target_pose_stamped)
            
            rospy.loginfo(f"下一个导航目标: x={x:.2f}, y={y:.2f}, yaw={yaw_deg}°")
            self.client.send_goal(goal)
            self.client.wait_for_result()

            status = self.client.get_state()
            if status == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("[成功] 已到达目标航点。")
            else:
                rospy.logwarn(f"[失败] 导航失败，状态码: {status}")
            
            # 在这里执行舞蹈动作
            rospy.loginfo("/* 开始执行指定动作... */")
            
            # 执行舞蹈动作
            self.perform_dance(goal)
            
            rospy.loginfo("/* 指定动作执行完毕。 */")
            
            if len(self.tried_waypoints) >= len(self.waypoints):
                rospy.loginfo("完成")
                break

        rospy.signal_shutdown("全部结束")


if __name__ == "__main__":
    try:
        rospy.init_node("nav_point_sequence")

        # 定义航点列表: (x, y, yaw_deg)
        # 绝对坐标      (建图时)
        waypoints = [
            (-5.238, -0.204, 117.677),
            (1.122, -0.413, -53.2),
        ]

        navigator = NavPointSequence(waypoints)
        navigator.run_sequence()
        
        rospy.spin()  # 保持节点运行，直到被外部关闭

    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断。")