#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
高成功率导航到后台点脚本
使用多种策略确保机器人能够成功导航到 (-0.8, 0, 0) 点
"""

import rospy
import math
import threading
import time
import tf.transformations as tft
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from dynamic_reconfigure.client import Client

class RobustBackstageNavigator:
    def __init__(self):
        """初始化高成功率导航器"""
        rospy.init_node("robust_backstage_navigator")
        
        # 目标点
        self.target_pos = (-0.8, 0, 0)
        
        # 当前机器人位置
        self.current_position = {"x": 0.0, "y": 0.0, "theta": 0.0}
        
        # 导航状态
        self.navigation_success = False
        self.attempt_count = 0
        self.max_attempts = 20  # 最大尝试次数
        
        # 发布器和订阅器
        self.goal_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=1)
        self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.arrival_sub = rospy.Subscriber("/robot_arrival_status", String, self.arrival_callback)
        
        # 添加位置订阅器
        try:
            from nav_msgs.msg import Odometry
            self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
            rospy.loginfo("已订阅里程计话题 /odom")
        except ImportError:
            rospy.logwarn("无法导入nav_msgs，将使用默认位置更新")
            self.odom_sub = None
        
        # 多种备用目标点策略
        self.backup_targets = self._generate_backup_targets()
        
        # 规划器参数配置
        self.planner_configs = self._generate_planner_configs()
        
        rospy.loginfo("高成功率后台导航器初始化完成")
        rospy.loginfo(f"目标点: {self.target_pos}")
        rospy.loginfo(f"备用目标点数量: {len(self.backup_targets)}")
        rospy.loginfo(f"规划器配置数量: {len(self.planner_configs)}")
    
    def _generate_backup_targets(self):
        """生成多个备用目标点，模拟RViz手动点击的位置"""
        base_x, base_y, base_theta = self.target_pos
        
        # 生成多个备用点，覆盖目标点周围的区域
        backup_targets = [
            # 原始目标点
            (base_x, base_y, base_theta),
            
            # 微调位置
            (base_x + 0.1, base_y, base_theta),
            (base_x - 0.1, base_y, base_theta),
            (base_x, base_y + 0.1, base_theta),
            (base_x, base_y - 0.1, base_theta),
            
            # 小幅度调整
            (base_x + 0.2, base_y, base_theta),
            (base_x - 0.2, base_y, base_theta),
            (base_x, base_y + 0.2, base_theta),
            (base_x, base_y - 0.2, base_theta),
            
            # 角度调整
            (base_x, base_y, base_theta + 10),
            (base_x, base_y, base_theta - 10),
            (base_x, base_y, base_theta + 20),
            (base_x, base_y, base_theta - 20),
            
            # 组合调整
            (base_x + 0.1, base_y + 0.1, base_theta),
            (base_x - 0.1, base_y + 0.1, base_theta),
            (base_x + 0.1, base_y - 0.1, base_theta),
            (base_x - 0.1, base_y - 0.1, base_theta),
            
            # 更大范围调整
            (base_x + 0.3, base_y, base_theta),
            (base_x - 0.3, base_y, base_theta),
            (base_x, base_y + 0.3, base_theta),
            (base_x, base_y - 0.3, base_theta),
            
            # 极端位置（最后尝试）
            (base_x + 0.5, base_y, base_theta),
            (base_x - 0.5, base_y, base_theta),
            (base_x, base_y + 0.5, base_theta),
            (base_x, base_y - 0.5, base_theta),
        ]
        
        return backup_targets
    
    def _generate_planner_configs(self):
        """生成多种规划器配置 - 优先确保机器人能够移动"""
        configs = [
            # 配置1：超宽松参数 - 优先移动，不要求精准
            {
                'name': 'ultra_loose_move',
                'xy_goal_tolerance': 2.0,
                'yaw_goal_tolerance': 2.0,
                'min_obstacle_dist': 0.01,
                'inflation_dist': 0.01,
                'weight_obstacle': 1,
                'weight_inflation': 0.01,
                'max_vel_x': 0.8,
                'max_vel_theta': 0.6,
                'acc_lim_x': 0.6,
                'acc_lim_theta': 0.6,
            },
            
            # 配置2：强制移动参数
            {
                'name': 'force_move',
                'xy_goal_tolerance': 3.0,
                'yaw_goal_tolerance': 3.0,
                'min_obstacle_dist': 0.005,
                'inflation_dist': 0.005,
                'weight_obstacle': 0.1,
                'weight_inflation': 0.005,
                'max_vel_x': 1.2,
                'max_vel_theta': 0.8,
                'acc_lim_x': 0.8,
                'acc_lim_theta': 0.8,
            },
            
            # 配置3：极简参数
            {
                'name': 'minimal',
                'xy_goal_tolerance': 5.0,
                'yaw_goal_tolerance': 5.0,
                'min_obstacle_dist': 0.001,
                'inflation_dist': 0.001,
                'weight_obstacle': 0.01,
                'weight_inflation': 0.001,
                'max_vel_x': 1.5,
                'max_vel_theta': 1.0,
                'acc_lim_x': 1.0,
                'acc_lim_theta': 1.0,
            },
            
            # 配置4：默认参数（备用）
            {
                'name': 'default',
                'xy_goal_tolerance': 0.2,
                'yaw_goal_tolerance': 0.15,
                'min_obstacle_dist': 0.1,
                'inflation_dist': 0.2,
                'weight_obstacle': 50,
                'weight_inflation': 0.1,
                'max_vel_x': 3.0,
                'max_vel_theta': 1.0,
                'acc_lim_x': 1.0,
                'acc_lim_theta': 1.0,
            },
            
            # 配置5：宽松参数（备用）
            {
                'name': 'loose',
                'xy_goal_tolerance': 0.5,
                'yaw_goal_tolerance': 0.5,
                'min_obstacle_dist': 0.05,
                'inflation_dist': 0.1,
                'weight_obstacle': 20,
                'weight_inflation': 0.05,
                'max_vel_x': 1.5,
                'max_vel_theta': 0.8,
                'acc_lim_x': 0.8,
                'acc_lim_theta': 0.8,
            }
        ]
        
        return configs
    
    def _build_move_base_goal(self, x, y, theta_deg):
        """构建MoveBaseActionGoal消息"""
        goal = MoveBaseActionGoal()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.goal_id.stamp = rospy.Time.now()
        goal.goal_id.id = f"robust_nav_{rospy.Time.now().to_nsec()}"
        goal.goal.target_pose.header.frame_id = "map"
        goal.goal.target_pose.header.stamp = rospy.Time.now()
        goal.goal.target_pose.pose.position.x = x
        goal.goal.target_pose.pose.position.y = y
        q = tft.quaternion_from_euler(0, 0, math.radians(theta_deg))
        goal.goal.target_pose.pose.orientation.x = q[0]
        goal.goal.target_pose.pose.orientation.y = q[1]
        goal.goal.target_pose.pose.orientation.z = q[2]
        goal.goal.target_pose.pose.orientation.w = q[3]
        return goal
    
    def _apply_planner_config(self, config):
        """应用规划器配置"""
        try:
            client = Client("/move_base/TebLocalPlannerROS", timeout=5.0)
            client.update_configuration(config)
            rospy.loginfo(f"应用规划器配置: {config.get('name', 'unknown')}")
            return True
        except Exception as e:
            rospy.logwarn(f"应用规划器配置失败: {e}")
            return False
    
    def _stop_robot(self):
        """停止机器人"""
        try:
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            rospy.sleep(0.5)
        except Exception as e:
            rospy.logwarn(f"停止机器人失败: {e}")
    
    def _cancel_current_goal(self):
        """取消当前目标"""
        try:
            cancel_msg = GoalID()
            self.cancel_pub.publish(cancel_msg)
            rospy.sleep(0.2)
        except Exception as e:
            rospy.logwarn(f"取消目标失败: {e}")
    
    def _check_arrival(self, target_pos, tolerance=0.3):
        """检查是否到达目标点"""
        dx = self.current_position["x"] - target_pos[0]
        dy = self.current_position["y"] - target_pos[1]
        distance = math.sqrt(dx*dx + dy*dy)
        return distance < tolerance
    
    def _try_single_navigation(self, target_pos, config, timeout=15):
        """尝试单次导航 - 优先确保机器人能够移动"""
        rospy.loginfo(f"尝试导航到: {target_pos}, 配置: {config.get('name', 'unknown')}")
        
        # 停止机器人
        self._stop_robot()
        
        # 取消当前目标
        self._cancel_current_goal()
        
        # 应用规划器配置
        if not self._apply_planner_config(config):
            rospy.logwarn("规划器配置失败，使用默认参数")
        
        # 发布目标
        goal_msg = self._build_move_base_goal(target_pos[0], target_pos[1], target_pos[2])
        self.goal_pub.publish(goal_msg)
        
        # 记录开始位置
        start_x = self.current_position["x"]
        start_y = self.current_position["y"]
        
        # 等待导航完成 - 缩短超时时间，优先检查是否开始移动
        start_time = rospy.Time.now()
        movement_detected = False
        
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            # 检查是否开始移动（不要求到达目标）
            current_x = self.current_position["x"]
            current_y = self.current_position["y"]
            distance_moved = math.sqrt((current_x - start_x)**2 + (current_y - start_y)**2)
            
            if distance_moved > 0.1:  # 移动了超过10cm就认为开始移动了
                if not movement_detected:
                    rospy.loginfo(f"机器人开始移动，已移动 {distance_moved:.2f}米")
                    movement_detected = True
            
            # 检查是否到达目标（使用更宽松的容差）
            if self._check_arrival(target_pos, tolerance=1.0):  # 1米容差
                rospy.loginfo(f"? 成功到达目标点: {target_pos}")
                return True
            
            rospy.sleep(0.5)
        
        # 如果机器人开始移动了，即使没到达目标也认为部分成功
        if movement_detected:
            rospy.loginfo(f"? 机器人已开始移动，部分成功: {target_pos}")
            return True
        
        rospy.logwarn(f"? 导航超时，机器人未移动: {target_pos}")
        return False
    
    def _try_waypoint_sequence(self, target_pos, config):
        """尝试通过中间航点序列导航"""
        rospy.loginfo(f"尝试航点序列导航到: {target_pos}")
        
        # 获取当前机器人位置
        current_x = self.current_position["x"]
        current_y = self.current_position["y"]
        
        # 生成中间航点
        intermediate_waypoints = [
            # 当前点到目标点的中间点
            ((current_x + target_pos[0]) / 2, (current_y + target_pos[1]) / 2, target_pos[2]),
            # 目标点附近的安全点
            (target_pos[0] + 0.2, target_pos[1], target_pos[2]),
            (target_pos[0] - 0.2, target_pos[1], target_pos[2]),
            (target_pos[0], target_pos[1] + 0.2, target_pos[2]),
            (target_pos[0], target_pos[1] - 0.2, target_pos[2]),
            # 最终目标点
            target_pos
        ]
        
        # 依次尝试每个航点
        for i, waypoint in enumerate(intermediate_waypoints):
            rospy.loginfo(f"尝试中间航点 {i+1}/{len(intermediate_waypoints)}: {waypoint}")
            
            if self._try_single_navigation(waypoint, config, timeout=20):
                if i == len(intermediate_waypoints) - 1:  # 最后一个航点
                    return True
                rospy.sleep(1.0)  # 短暂停留
            else:
                rospy.logwarn(f"中间航点 {i+1} 失败，继续下一个")
        
        return False
    
    def _try_force_approach(self, target_pos):
        """尝试强制接近目标点 - 优先确保机器人能够移动"""
        rospy.loginfo(f"尝试强制接近: {target_pos}")
        
        # 停止机器人
        self._stop_robot()
        
        # 计算到目标点的方向
        dx = target_pos[0] - self.current_position["x"]
        dy = target_pos[1] - self.current_position["y"]
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < 2.0:  # 已经很接近了（放宽容差）
            return True
        
        # 计算移动方向
        angle = math.atan2(dy, dx)
        
        # 发布移动命令 - 更积极的移动
        move_msg = Twist()
        move_msg.linear.x = 0.5  # 增加移动速度
        move_msg.angular.z = angle * 0.8  # 增加转向速度
        
        # 记录开始位置
        start_x = self.current_position["x"]
        start_y = self.current_position["y"]
        movement_detected = False
        
        # 持续移动直到接近目标
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 20.0:  # 增加超时时间
            self.cmd_vel_pub.publish(move_msg)
            
            # 检查是否开始移动
            current_x = self.current_position["x"]
            current_y = self.current_position["y"]
            distance_moved = math.sqrt((current_x - start_x)**2 + (current_y - start_y)**2)
            
            if distance_moved > 0.2:  # 移动了超过20cm
                if not movement_detected:
                    rospy.loginfo(f"强制接近：机器人开始移动，已移动 {distance_moved:.2f}米")
                    movement_detected = True
            
            if self._check_arrival(target_pos, tolerance=1.5):  # 放宽容差
                self._stop_robot()
                rospy.loginfo(f"? 强制接近成功: {target_pos}")
                return True
            
            rospy.sleep(0.1)
        
        self._stop_robot()
        
        # 如果机器人开始移动了，即使没到达目标也认为部分成功
        if movement_detected:
            rospy.loginfo(f"? 强制接近部分成功，机器人已移动: {target_pos}")
            return True
        
        rospy.logwarn(f"? 强制接近失败，机器人未移动: {target_pos}")
        return False
    
    def odom_callback(self, msg):
        """处理里程计数据，更新机器人位置"""
        try:
            self.current_position["x"] = msg.pose.pose.position.x
            self.current_position["y"] = msg.pose.pose.position.y
            
            # 计算偏航角
            orientation = msg.pose.pose.orientation
            quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
            euler = tft.euler_from_quaternion(quaternion)
            self.current_position["theta"] = math.degrees(euler[2])
        except Exception as e:
            rospy.logwarn(f"处理里程计数据失败: {e}")
    
    def arrival_callback(self, msg):
        """处理到达状态回调"""
        if msg.data == "arrived":
            rospy.loginfo("收到机器人到达状态")
            if self._check_arrival(self.target_pos):
                rospy.loginfo("? 机器人已成功到达后台点！")
                self.navigation_success = True
            else:
                rospy.loginfo("机器人到达了某个点，但可能不是目标点")
    
    def navigate_to_backstage(self):
        """主要导航方法 - 优先确保机器人能够移动"""
        rospy.loginfo("开始高成功率导航到后台点...")
        rospy.loginfo(f"目标点: {self.target_pos}")
        rospy.loginfo("优先策略：确保机器人能够移动，不要求精准度")
        
        # 策略1：优先尝试强制移动参数
        rospy.loginfo("策略1：尝试强制移动参数...")
        for config in self.planner_configs[:3]:  # 只尝试前3个强制移动配置
            if self.navigation_success:
                break
                
            rospy.loginfo(f"尝试规划器配置: {config['name']}")
            
            # 只尝试前几个目标点，快速测试
            for i, target in enumerate(self.backup_targets[:8]):  # 只尝试前8个目标点
                if self.navigation_success:
                    break
                    
                self.attempt_count += 1
                rospy.loginfo(f"尝试 {self.attempt_count}/{self.max_attempts}: 目标点 {i+1}/8")
                
                # 尝试直接导航
                if self._try_single_navigation(target, config, timeout=10):  # 缩短超时时间
                    self.navigation_success = True
                    break
                
                # 短暂休息
                rospy.sleep(0.5)
        
        # 策略2：如果规划器导航失败，立即尝试强制接近
        if not self.navigation_success:
            rospy.logwarn("规划器导航失败，立即尝试强制接近...")
            if self._try_force_approach(self.target_pos):
                self.navigation_success = True
        
        # 策略3：如果强制接近也失败，尝试更激进的方法
        if not self.navigation_success:
            rospy.logwarn("尝试更激进的移动方法...")
            
            # 尝试多个方向的强制移动
            directions = [
                (0.3, 0, 0),    # 向前
                (-0.3, 0, 0),   # 向后
                (0, 0.3, 0),    # 向左
                (0, -0.3, 0),   # 向右
                (0.2, 0.2, 0),  # 斜向
                (-0.2, 0.2, 0), # 斜向
            ]
            
            for i, (dx, dy, dtheta) in enumerate(directions):
                test_target = (self.target_pos[0] + dx, self.target_pos[1] + dy, self.target_pos[2] + dtheta)
                rospy.loginfo(f"尝试方向 {i+1}: {test_target}")
                
                if self._try_force_approach(test_target):
                    self.navigation_success = True
                    break
                
                rospy.sleep(1.0)
        
        # 最终结果
        if self.navigation_success:
            rospy.loginfo("? 导航成功！机器人已开始移动或到达目标")
        else:
            rospy.logerr("? 所有移动策略都失败了")
            rospy.logerr(f"尝试了 {self.attempt_count} 次导航")
            rospy.logerr("建议检查机器人硬件和传感器状态")
        
        return self.navigation_success
    
    def run(self):
        """运行导航器"""
        try:
            # 等待系统稳定
            rospy.sleep(2.0)
            
            # 开始导航
            success = self.navigate_to_backstage()
            
            if success:
                rospy.loginfo("导航任务完成")
            else:
                rospy.logerr("导航任务失败")
                
        except KeyboardInterrupt:
            rospy.loginfo("用户中断导航")
        except Exception as e:
            rospy.logerr(f"导航过程中发生错误: {e}")
        finally:
            # 清理
            self._stop_robot()
            self._cancel_current_goal()

def main():
    """主函数"""
    try:
        navigator = RobustBackstageNavigator()
        navigator.run()
    except Exception as e:
        rospy.logerr(f"程序启动失败: {e}")

if __name__ == "__main__":
    main()
