#!/usr/bin/env python3
"""
测试后台点RViz风格备用策略
"""

import rospy
import math
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback
from actionlib_msgs.msg import GoalID, GoalStatusArray

class BackstageTest:
    def __init__(self):
        rospy.init_node('backstage_test')
        
        # 发布器
        self.goal_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=1)
        self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        # 订阅器
        self.feedback_sub = rospy.Subscriber("/move_base/feedback", MoveBaseActionFeedback, self.feedback_callback)
        self.status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_callback)
        
        # 状态
        self.current_position = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.test_active = False
        
        rospy.loginfo("后台点测试已启动")
        
    def feedback_callback(self, msg):
        """更新当前位置"""
        current_pose = msg.feedback.base_position.pose
        self.current_position["x"] = current_pose.position.x
        self.current_position["y"] = current_pose.position.y
        
        orientation = current_pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = tft.euler_from_quaternion(quaternion)
        self.current_position["theta"] = math.degrees(euler[2])
        
    def status_callback(self, msg):
        """处理导航状态"""
        if msg.status_list:
            status = msg.status_list[0]
            status_names = {0: "PENDING", 1: "ACTIVE", 2: "PREEMPTED", 3: "SUCCEEDED", 
                          4: "ABORTED", 5: "REJECTED", 6: "PREEMPTING", 7: "RECALLING", 
                          8: "RECALLED", 9: "LOST"}
            status_name = status_names.get(status.status, "UNKNOWN")
            rospy.loginfo(f"导航状态: {status_name}")
            
    def create_goal(self, x, y, theta_deg):
        """创建导航目标"""
        goal = MoveBaseActionGoal()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.goal_id.stamp = rospy.Time.now()
        goal.goal_id.id = f"backstage_test_{int(rospy.Time.now().to_nsec())}"
        
        goal.goal.target_pose.header.frame_id = "map"
        goal.goal.target_pose.header.stamp = rospy.Time.now()
        goal.goal.target_pose.pose.position.x = x
        goal.goal.target_pose.pose.position.y = y
        goal.goal.target_pose.pose.position.z = 0.0
        
        q = tft.quaternion_from_euler(0, 0, math.radians(theta_deg))
        goal.goal.target_pose.pose.orientation.x = q[0]
        goal.goal.target_pose.pose.orientation.y = q[1]
        goal.goal.target_pose.pose.orientation.z = q[2]
        goal.goal.target_pose.pose.orientation.w = q[3]
        
        return goal
        
    def try_rviz_style_backstage(self):
        """测试RViz风格的后台点策略"""
        rospy.loginfo("开始RViz风格后台点测试...")
        
        # 原始后台点
        original_backstage = (-0.6, 0, 0)
        
        # 备用后台点列表
        alternative_points = [
            (-0.5, 0, 0),      # 稍微向右
            (-0.7, 0, 0),      # 稍微向左  
            (-0.6, 0.1, 0),    # 稍微向前
            (-0.6, -0.1, 0),   # 稍微向后
            (-0.4, 0, 0),      # 更向右
            (-0.8, 0, 0),      # 更向左
        ]
        
        # 先尝试原始后台点
        rospy.loginfo(f"尝试原始后台点: {original_backstage}")
        self._test_single_point(original_backstage, "原始后台点")
        
        # 如果失败，尝试备用点
        for i, (x, y, theta) in enumerate(alternative_points):
            rospy.loginfo(f"尝试备用点 {i+1}: ({x}, {y}, {theta})")
            success = self._test_single_point((x, y, theta), f"备用点{i+1}")
            if success:
                rospy.loginfo(f"✅ 备用点 {i+1} 成功!")
                return True
                
        rospy.logwarn("所有备用点都失败")
        return False
        
    def _test_single_point(self, point, name):
        """测试单个点"""
        x, y, theta = point
        
        # 取消当前目标
        cancel_msg = GoalID()
        self.cancel_pub.publish(cancel_msg)
        rospy.sleep(0.5)
        
        # 发布目标
        goal_msg = self.create_goal(x, y, theta)
        self.goal_pub.publish(goal_msg)
        rospy.loginfo(f"发布{name}目标: ({x}, {y}, {theta})")
        
        # 等待并检查结果
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 10.0:  # 10秒超时
            # 检查距离
            dx = self.current_position["x"] - (-0.6)
            dy = self.current_position["y"] - 0.0
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < 0.5:  # 如果接近后台点
                rospy.loginfo(f"✅ {name}成功，距离后台点 {distance:.2f}米")
                return True
                
            rospy.sleep(0.5)
            
        rospy.logwarn(f"❌ {name}超时")
        return False
        
    def run_test(self):
        """运行测试"""
        rospy.loginfo("等待系统稳定...")
        rospy.sleep(3)
        
        rospy.loginfo("开始后台点RViz风格测试...")
        success = self.try_rviz_style_backstage()
        
        if success:
            rospy.loginfo("🎉 测试成功!")
        else:
            rospy.logwarn("⚠️  测试失败")
            
        rospy.loginfo("测试完成")

def main():
    test = BackstageTest()
    test.run_test()
    rospy.spin()

if __name__ == '__main__':
    main()
