#!/usr/bin/env python3
"""
导航修复测试脚本
用于验证路径规划修复是否有效
"""

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import tf2_ros
import tf2_geometry_msgs
import math

class NavigationTester:
    def __init__(self):
        rospy.init_node('navigation_tester')
        
        # 创建move_base客户端
        self.move_base_client = actionlib.SimpleActionClient('move_base_flex/move_base', MoveBaseAction)
        
        # 等待服务器
        rospy.loginfo("等待move_base服务器...")
        if not self.move_base_client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("无法连接到move_base服务器")
            return
            
        rospy.loginfo("已连接到move_base服务器")
        
        # TF监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
    def get_current_pose(self):
        """获取当前机器人位置"""
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            
            return pose
        except Exception as e:
            rospy.logerr(f"无法获取当前位置: {e}")
            return None
            
    def create_goal(self, x, y, yaw=0.0):
        """创建导航目标"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        
        # 设置方向
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return goal
        
    def test_short_distance(self):
        """测试短距离导航"""
        rospy.loginfo("=== 测试短距离导航 ===")
        
        current_pose = self.get_current_pose()
        if current_pose is None:
            rospy.logerr("无法获取当前位置，跳过测试")
            return False
            
        # 创建距离当前位置1米的目标
        goal_x = current_pose.pose.position.x + 1.0
        goal_y = current_pose.pose.position.y
        
        goal = self.create_goal(goal_x, goal_y)
        
        rospy.loginfo(f"发送目标: ({goal_x:.2f}, {goal_y:.2f})")
        
        # 发送目标
        self.move_base_client.send_goal(goal)
        
        # 等待结果
        result = self.move_base_client.wait_for_result(rospy.Duration(30.0))
        
        if result:
            state = self.move_base_client.get_state()
            if state == 3:  # SUCCEEDED
                rospy.loginfo("? 短距离导航成功!")
                return True
            else:
                rospy.logwarn(f"? 短距离导航失败，状态: {state}")
                return False
        else:
            rospy.logwarn("? 短距离导航超时")
            return False
            
    def test_medium_distance(self):
        """测试中距离导航"""
        rospy.loginfo("=== 测试中距离导航 ===")
        
        current_pose = self.get_current_pose()
        if current_pose is None:
            rospy.logerr("无法获取当前位置，跳过测试")
            return False
            
        # 创建距离当前位置3米的目标
        goal_x = current_pose.pose.position.x + 3.0
        goal_y = current_pose.pose.position.y
        
        goal = self.create_goal(goal_x, goal_y)
        
        rospy.loginfo(f"发送目标: ({goal_x:.2f}, {goal_y:.2f})")
        
        # 发送目标
        self.move_base_client.send_goal(goal)
        
        # 等待结果
        result = self.move_base_client.wait_for_result(rospy.Duration(60.0))
        
        if result:
            state = self.move_base_client.get_state()
            if state == 3:  # SUCCEEDED
                rospy.loginfo("? 中距离导航成功!")
                return True
            else:
                rospy.logwarn(f"? 中距离导航失败，状态: {state}")
                return False
        else:
            rospy.logwarn("? 中距离导航超时")
            return False
            
    def test_rotation(self):
        """测试原地旋转"""
        rospy.loginfo("=== 测试原地旋转 ===")
        
        current_pose = self.get_current_pose()
        if current_pose is None:
            rospy.logerr("无法获取当前位置，跳过测试")
            return False
            
        # 创建原地旋转90度的目标
        goal = self.create_goal(
            current_pose.pose.position.x,
            current_pose.pose.position.y,
            math.pi / 2  # 90度
        )
        
        rospy.loginfo("发送旋转目标: 90度")
        
        # 发送目标
        self.move_base_client.send_goal(goal)
        
        # 等待结果
        result = self.move_base_client.wait_for_result(rospy.Duration(30.0))
        
        if result:
            state = self.move_base_client.get_state()
            if state == 3:  # SUCCEEDED
                rospy.loginfo("? 原地旋转成功!")
                return True
            else:
                rospy.logwarn(f"? 原地旋转失败，状态: {state}")
                return False
        else:
            rospy.logwarn("? 原地旋转超时")
            return False
            
    def run_all_tests(self):
        """运行所有测试"""
        rospy.loginfo("开始导航修复测试...")
        
        tests = [
            ("短距离导航", self.test_short_distance),
            ("原地旋转", self.test_rotation),
            ("中距离导航", self.test_medium_distance),
        ]
        
        results = {}
        
        for test_name, test_func in tests:
            rospy.loginfo(f"\n--- 开始 {test_name} ---")
            try:
                result = test_func()
                results[test_name] = result
                
                if result:
                    rospy.loginfo(f"? {test_name} 通过")
                else:
                    rospy.logwarn(f"? {test_name} 失败")
                    
            except Exception as e:
                rospy.logerr(f"? {test_name} 出错: {e}")
                results[test_name] = False
                
            # 测试间隔
            rospy.sleep(2)
            
        # 输出测试结果
        rospy.loginfo("\n=== 测试结果汇总 ===")
        for test_name, result in results.items():
            status = "? 通过" if result else "? 失败"
            rospy.loginfo(f"{test_name}: {status}")
            
        passed = sum(results.values())
        total = len(results)
        rospy.loginfo(f"总计: {passed}/{total} 测试通过")
        
        if passed == total:
            rospy.loginfo("? 所有测试通过！导航修复成功！")
        else:
            rospy.logwarn("??  部分测试失败，可能需要进一步调整参数")

def main():
    tester = NavigationTester()
    
    if tester.move_base_client is None:
        rospy.logerr("无法初始化导航测试器")
        return
        
    # 等待系统稳定
    rospy.sleep(3)
    
    # 运行测试
    tester.run_all_tests()

if __name__ == '__main__':
    main()
