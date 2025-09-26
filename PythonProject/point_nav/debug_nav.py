#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID
import tf.transformations as tft
import math

def test_navigation():
    """测试导航系统是否正常工作"""
    rospy.init_node("debug_navigation")
    
    # 创建发布者
    goal_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=1)
    cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
    
    rospy.loginfo("等待move_base节点...")
    rospy.sleep(2.0)
    
    # 测试目标点（第一个舞蹈点）
    test_goal = (-2.3, 3.4, 170)
    x, y, theta = test_goal
    
    rospy.loginfo(f"测试导航到: x={x}, y={y}, theta={theta}")
    
    # 构建目标消息
    goal = MoveBaseActionGoal()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"
    goal.goal_id.stamp = rospy.Time.now()
    goal.goal_id.id = f"test_goal_{rospy.Time.now().to_nsec()}"
    goal.goal.target_pose.header.frame_id = "map"
    goal.goal.target_pose.header.stamp = rospy.Time.now()
    goal.goal.target_pose.pose.position.x = x
    goal.goal.target_pose.pose.position.y = y
    q = tft.quaternion_from_euler(0, 0, math.radians(theta))
    goal.goal.target_pose.pose.orientation.x = q[0]
    goal.goal.target_pose.pose.orientation.y = q[1]
    goal.goal.target_pose.pose.orientation.z = q[2]
    goal.goal.target_pose.pose.orientation.w = q[3]
    
    # 发布目标
    rospy.loginfo("发布测试目标...")
    goal_pub.publish(goal)
    
    # 等待一段时间
    rospy.loginfo("等待10秒观察结果...")
    rospy.sleep(10.0)
    
    # 取消目标
    rospy.loginfo("取消测试目标...")
    cancel_msg = GoalID()
    cancel_pub.publish(cancel_msg)
    
    rospy.loginfo("测试完成")

if __name__ == "__main__":
    try:
        test_navigation()
    except rospy.ROSInterruptException:
        pass
