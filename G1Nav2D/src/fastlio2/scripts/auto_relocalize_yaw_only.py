# -*- coding: utf-8 -*-
#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
import time
import subprocess
import re
from move_base_msgs.msg import MoveBaseActionFeedback

class AutoRelocalizeYawOnly:
    def __init__(self):
        rospy.init_node('auto_relocalize_yaw_only', anonymous=True)
        self.pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        self.success_threshold = 0.03
        self.wait_time = 1.0
        self.yaw_list = [0, 1.57, 3.14, -1.57]  # 可根据实际情况添加更多角度
        self.successful = False
        self.current_pose = None
        rospy.Subscriber("/move_base/feedback", MoveBaseActionFeedback, self.move_base_feedback_cb)

    def move_base_feedback_cb(self, msg):
        # 提取反馈中的base_position
        self.current_pose = msg.feedback.base_position.pose

    def create_pose_msg(self, x, y, yaw):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose_msg.pose.pose.orientation.x = quaternion[0]
        pose_msg.pose.pose.orientation.y = quaternion[1]
        pose_msg.pose.pose.orientation.z = quaternion[2]
        pose_msg.pose.pose.orientation.w = quaternion[3]
        pose_msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        return pose_msg

    def check_icp_success(self):
        try:
            cmd = "tail -n 50 ~/.ros/log/latest/stdout.log | grep -A 5 'ICP'"
            result = subprocess.check_output(cmd, shell=True, text=True)
            if "Alignment SUCCESS" in result:
                match = re.search(r"Refine score: ([0-9.]+)", result)
                if match:
                    score = float(match.group(1))
                    if score < self.success_threshold:
                        rospy.loginfo(f"ICP成功! 分数: {score}")
                        return True
            return False
        except Exception as e:
            rospy.logwarn(f"检查ICP状态时发生错误: {e}")
            return False

    def run(self):
        rospy.loginfo("开始仅调整yaw的自动重定位过程...")
        while not rospy.is_shutdown() and not self.successful:
            if self.current_pose is None:
                rospy.loginfo("等待当前位置数据...")
                rospy.sleep(0.5)
                continue
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            for yaw in self.yaw_list:
                rospy.loginfo(f"尝试姿态估计: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
                pose_msg = self.create_pose_msg(x, y, yaw)
                self.pose_pub.publish(pose_msg)
                time.sleep(self.wait_time)
                try:
                    self.clear_costmaps()
                    rospy.loginfo("成功清除代价地图!")
                except rospy.ServiceException as e:
                    rospy.logwarn(f"清除代价地图失败: {e}")
                time.sleep(self.wait_time)
                if self.check_icp_success():
                    self.successful = True
                    rospy.loginfo(f"重定位成功! yaw={yaw:.2f}")
                    break
            if not self.successful:
                rospy.logwarn("所有yaw尝试均失败，等待再次尝试...")
                time.sleep(2)
        if self.successful:
            rospy.loginfo("自动重定位成功")
        else:
            rospy.logwarn("重定位失败")

if __name__ == '__main__':
    try:
        relocalize = AutoRelocalizeYawOnly()
        relocalize.run()
    except rospy.ROSInterruptException:
        pass