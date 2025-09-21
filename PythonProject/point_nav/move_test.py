#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Quaternion
from std_srvs.srv import SetBool
import tf.transformations
import math
import sys


class WaypointNavigation:
    """
    导航程序，用于控制机器人按照预定义航点进行导航
    并在特定航点之间切换旋转控制设置
    """

    def __init__(self):
        rospy.init_node("waypoint_navigation", anonymous=False)

        # 定义航点列表 [x, y, theta(弧度)]
        self.waypoints = [
            [0.0, 0.0, 0.0],  # 起始点 (索引0)
            [2.0, 0.0, 0.0],  # 航点1
            [4.0, 0.0, math.pi / 4],  # 航点2
            [6.0, 2.0, 0.0],  # 航点3
            [8.0, 2.0, 0.0],  # 航点4
            [10.0, 0.0, -math.pi / 2],  # 航点5
        ]

        self.current_waypoint_index = 0
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.controller_ns = rospy.get_param(
            "~controller_ns", "/unitree_cmd_vel_controller"
        )

        # 等待旋转控制服务可用
        service_name = f"{self.controller_ns}/set_rotation_enabled"
        rospy.loginfo(f"Waiting for rotation control service: {service_name}")
        try:
            rospy.wait_for_service(service_name, timeout=10.0)
            self.set_rotation = rospy.ServiceProxy(service_name, SetBool)
            rospy.loginfo("Rotation control service is ready")
        except rospy.ROSException as e:
            rospy.logerr(f"Rotation control service not available: {e}")
            rospy.logerr(
                "Make sure g1_ctrl_cwk.py is running with the correct namespace"
            )
            sys.exit(1)

        # 连接到move_base行动服务器
        self.move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )
        rospy.loginfo("Waiting for move_base action server...")
        if not self.move_base_client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("move_base action server not available!")
            sys.exit(1)
        rospy.loginfo("move_base action server is ready")

        # 发布当前目标点的可视化
        self.goal_pub = rospy.Publisher("/current_goal", PoseStamped, queue_size=1)

        rospy.loginfo("Waypoint navigation system initialized successfully!")

    def create_goal(self, waypoint):
        """从航点坐标创建MoveBaseGoal对象"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = waypoint[0]
        goal.target_pose.pose.position.y = waypoint[1]
        goal.target_pose.pose.position.z = 0.0

        # 从偏航角创建四元数
        q = tf.transformations.quaternion_from_euler(0, 0, waypoint[2])
        goal.target_pose.pose.orientation = Quaternion(*q)

        return goal

    def publish_goal(self, goal):
        """发布当前目标点以便可视化"""
        self.goal_pub.publish(goal.target_pose)

    def navigate_to_waypoint(self, waypoint_index):
        """导航到指定索引的航点"""
        if waypoint_index < 0 or waypoint_index >= len(self.waypoints):
            rospy.logerr(f"Invalid waypoint index: {waypoint_index}")
            return False

        # 根据当前航点和目标航点设置旋转控制
        self.set_rotation_flag(self.current_waypoint_index, waypoint_index)

        # 创建并发送导航目标
        waypoint = self.waypoints[waypoint_index]
        goal = self.create_goal(waypoint)
        self.publish_goal(goal)

        rospy.loginfo(
            f"Navigating from waypoint {self.current_waypoint_index} to waypoint {waypoint_index}: "
            f"[{waypoint[0]:.2f}, {waypoint[1]:.2f}, {waypoint[2]:.2f}]"
        )

        # 发送目标并等待完成
        self.move_base_client.send_goal(goal)
        result = self.move_base_client.wait_for_result()

        if result:
            rospy.loginfo(f"Reached waypoint {waypoint_index}")
            self.current_waypoint_index = waypoint_index
            return True
        else:
            rospy.logerr(f"Failed to reach waypoint {waypoint_index}")
            return False

    def set_rotation_flag(self, from_wp, to_wp):
        """根据航点转换设置旋转控制标志"""
        try:
            # 默认允许旋转
            enable_rotation = True

            # 根据航点转换规则确定旋转控制
            if from_wp == 1 and to_wp == 2:
                # 从航点1到航点2：允许旋转
                enable_rotation = True
                rospy.loginfo("Waypoint 1->2: Enabling rotation (flag_rotate=1)")
            elif (from_wp == 2 and to_wp == 3) or (from_wp == 3 and to_wp == 4):
                # 从航点2到航点4：禁止旋转
                enable_rotation = False
                rospy.loginfo("Waypoint 2->4: Disabling rotation (flag_rotate=0)")
            elif from_wp == 4 and to_wp == 5:
                # 从航点4到航点5：允许旋转
                enable_rotation = True
                rospy.loginfo("Waypoint 4->5: Enabling rotation (flag_rotate=1)")

            # 调用服务设置旋转标志
            response = self.set_rotation(enable_rotation)
            if response.success:
                rospy.loginfo(
                    f"Successfully set rotation flag to {1 if enable_rotation else 0}"
                )
            else:
                rospy.logwarn(f"Failed to set rotation flag: {response.message}")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def execute_mission(self):
        """执行完整的导航任务，访问所有航点"""
        rospy.loginfo("Starting navigation mission...")

        # 从索引1开始（跳过起始点）
        for i in range(1, len(self.waypoints)):
            success = self.navigate_to_waypoint(i)
            if not success:
                rospy.logerr(f"Mission aborted at waypoint {i}")
                return False

            # 在航点之间暂停
            rospy.sleep(1.0)

        rospy.loginfo("Navigation mission completed successfully!")
        return True


if __name__ == "__main__":
    try:
        navigator = WaypointNavigation()
        navigator.execute_mission()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
