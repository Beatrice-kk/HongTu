import rospy
import math
import tf.transformations as tft
import actionlib
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry, GetPlan
from geometry_msgs.msg import PoseStamped

class NavPointSequence:
    def __init__(self, waypoints):
        self.waypoints = waypoints

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("waiting for move_base action server ...")
        self.client.wait_for_server()
        rospy.loginfo("move_base action server connected")

        rospy.wait_for_service('/move_base/make_plan')
        self.make_plan_srv = rospy.ServiceProxy('/move_base/make_plan', GetPlan)

        self.current_pose = None
        self.odom_sub = rospy.Subscriber('/slam_odom', Odometry, self.odom_callback, queue_size=10)

        # 等待odom
        rospy.sleep(1.0)
        self.run_sequence()

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def is_feasible(self, start_pose, goal_pose):
        try:
            resp = self.make_plan_srv(start=start_pose, goal=goal_pose, tolerance=0.2)
            if resp.plan.poses:
                return True
            else:
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"make_plan 调用失败: {e}")
            return False

    def build_pose_stamped(self, x, y, yaw_deg):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        q = tft.quaternion_from_euler(0, 0, math.radians(yaw_deg))
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def find_nearest_feasible_around(self, orig_wp, search_radius=0.3, step=0.1, angles=24):
        """在原航点附近采样，找可行点"""
        x0, y0, yaw = orig_wp
        start = PoseStamped()
        start.header.frame_id = "map"
        start.pose = self.current_pose

        # 先检查原点可行
        goal = self.build_pose_stamped(x0, y0, yaw)
        if self.is_feasible(start, goal):
            return (x0, y0, yaw)

        # 环形采样
        for r in np.arange(step, search_radius+step, step):
            for theta in np.linspace(0, 2 * math.pi, angles, endpoint=False):
                x = x0 + r * math.cos(theta)
                y = y0 + r * math.sin(theta)
                goal = self.build_pose_stamped(x, y, yaw)
                if self.is_feasible(start, goal):
                    rospy.loginfo(f"[补偿] 采样得新航点: x={x:.2f}, y={y:.2f}, yaw={yaw}")
                    return (x, y, yaw)
        rospy.logwarn(f"[补偿] 航点附近未找到可行点: x={x0:.2f}, y={y0:.2f}")
        return None

    def get_feasible_nearest_waypoint_with_adjust(self):
        """过滤不可行航点，若不可达则在附近采样补偿，返回最近的一个"""
        if self.current_pose is None:
            rospy.logwarn("等待里程计数据中...")
            return None

        feasible_points = []
        start = PoseStamped()
        start.header.frame_id = "map"
        start.pose = self.current_pose

        for wp in self.waypoints:
            # 先尝试原航点，否则采样
            adj_wp = self.find_nearest_feasible_around(wp)
            if adj_wp is not None:
                x, y, yaw = adj_wp
                dx = x - self.current_pose.position.x
                dy = y - self.current_pose.position.y
                dist = math.sqrt(dx*dx + dy*dy)
                feasible_points.append(((x, y, yaw), dist))

        if not feasible_points:
            return None

        feasible_points.sort(key=lambda p: p[1])
        return feasible_points[0][0]

    def run_sequence(self):
        tried = set()
        while len(tried) < len(self.waypoints):
            wp = self.get_feasible_nearest_waypoint_with_adjust()
            if wp is None:
                rospy.logwarn("没有可行的航点，任务结束")
                break

            if wp in tried:
                rospy.loginfo("最近的航点已经尝试过，结束")
                break

            tried.add(wp)
            x, y, yaw_deg = wp
            theta = math.radians(yaw_deg)

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            q = tft.quaternion_from_euler(0, 0, theta)
            goal.target_pose.pose.orientation.x = q[0]
            goal.target_pose.pose.orientation.y = q[1]
            goal.target_pose.pose.orientation.z = q[2]
            goal.target_pose.pose.orientation.w = q[3]

            rospy.loginfo(f"[导航目标] x={x:.2f}, y={y:.2f}, yaw={yaw_deg}°")

            #发布航点
            self.client.send_goal(goal)

            self.client.wait_for_result()
            status = self.client.get_state()
            if status == 3:  # SUCCEEDED
                rospy.loginfo("[到达] 成功到达该航点")
            else:
                rospy.logwarn(f"[到达] 导航失败，状态: {status}")
            
            
            rospy.loginfo('/* wait 一下*/')

            rospy.sleep(0.5)

            rospy.loginfo('/* wait 一下*/')

            rospy.sleep(0.5)



        rospy.loginfo("任务完成或无可行航点，退出程序")
        rospy.signal_shutdown("完成")


if __name__ == "__main__":
    rospy.init_node("nav_point_sequence")

    # 多个航点格式: (x, y, yaw_deg)
    waypoints = [
        (-1.1, 1.3, -90),
        (-0.5, 0.5, 0),
        (-1.1, 1.3, -180),
    ]

    NavPointSequence(waypoints)
    rospy.spin()