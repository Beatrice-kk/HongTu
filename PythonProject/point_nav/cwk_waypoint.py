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
class NavPointSequence:
    def __init__(self, waypoints):
        """
        初始化导航序列处理器。
        :param waypoints: 一个包含(x, y, yaw_deg)元组的列表。
        """
        self.waypoints = waypoints
        self.tried_waypoints = set()  # 存储已经尝试过的航点

        self.dance_index=0

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

    def odom_callback(self, msg: Odometry):
        """里程计回调函数，更新当前机器人位姿。"""
        self.current_pose = msg.pose.pose


    def is_feasible(self, start_pose: PoseStamped, goal_pose: PoseStamped) -> bool:
        """
         查看路径是否可行
        """
        try:
            # rospy.ServiceProxy 期望将请求的字段作为独立的参数传递，
            # 而不是一个请求对象。
            # 直接传递 start, goal, 和 tolerance
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
        """根据PoseStamped构建MoveBaseGoal消息。"""
        goal = MoveBaseGoal()
        goal.target_pose = pose_stamped
        return goal

    def find_nearest_feasible_around(self, orig_wp: tuple, search_radius=0.3, step=0.1, angles=24) -> Optional[tuple]:
        """
       环形采样 可行的航点

        """
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
        """
       寻找最近的可行航点。
        """
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
        """
         跳舞的时候 保持位置稳定
        """
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

    def run_sequence(self):
        
        while not rospy.is_shutdown():
            wp = self.get_feasible_nearest_waypoint()
            if wp is None:
                rospy.loginfo("没有更多可行的航点，任务结束。")
                break

            self.tried_waypoints.add(wp)
            x, y, yaw_deg = wp
            

            #构建并发送导航目标
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
            



            rospy.loginfo("/* 开始执行指定动作... */")


            self.stabilize_position(goal, rospy.Duration(3))



            rospy.loginfo("/* 指定动作执行完毕。 */")
            




            if len(self.tried_waypoints) >= len(self.waypoints):
                rospy.loginfo("完成")
                break

        rospy.signal_shutdown("全部结束shusuuuuuuuuuuuuuuuuuuuuu")


if __name__ == "__main__":
    try:
        rospy.init_node("nav_point_sequence")

        # 定义航点列表: (x, y, yaw_deg)

        #绝对坐标  （建图时）
        waypoints = [
            # (-1.8, -1.0, 180),
            (-0.3, 0.3, 180),

            # (-0.5, 0.5, 0),
            (-0.4, 0.2, 90),

            (-2.1, -0.1, 0),
        ]

        navigator = NavPointSequence(waypoints)
        navigator.run_sequence()
        
        rospy.spin() # 保持节点运行，直到被外部关闭

    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断。")
