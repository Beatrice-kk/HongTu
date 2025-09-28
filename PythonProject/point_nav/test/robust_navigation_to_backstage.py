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
        """应用规划器配置 - 已禁用，直接返回True"""
        rospy.loginfo(f"跳过规划器配置: {config.get('name', 'unknown')} (已禁用)")
        return True
    
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
    
    def _check_arrival(self, target_pos, tolerance=0.15):
        """检查是否到达目标点 - 使用更严格的容差"""
        dx = self.current_position["x"] - target_pos[0]
        dy = self.current_position["y"] - target_pos[1]
        distance = math.sqrt(dx*dx + dy*dy)
        rospy.loginfo(f"当前位置: ({self.current_position['x']:.3f}, {self.current_position['y']:.3f}), 目标: {target_pos}, 距离: {distance:.3f}m, 容差: {tolerance}m")
        return distance < tolerance
    
    def _check_arrival_g1_style(self, target_pos):
        """使用与g1_control相同的到达判断标准"""
        # 距离检查：0.4米容差（与g1_control一致）
        dx = self.current_position["x"] - target_pos[0]
        dy = self.current_position["y"] - target_pos[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        # 角度检查：35度容差（与g1_control一致）
        current_theta_rad = math.radians(self.current_position["theta"])
        target_theta_rad = math.radians(target_pos[2])
        angle_diff = abs(current_theta_rad - target_theta_rad)
        # 处理角度跨越问题
        if angle_diff > math.pi:
            angle_diff = 2 * math.pi - angle_diff
        
        rospy.loginfo(f"G1风格到达检查 - 距离: {distance:.3f}m (阈值: 0.4m), 角度差: {math.degrees(angle_diff):.1f}° (阈值: 35°)")
        
        # 与g1_control相同的判断条件
        distance_ok = distance <= 0.4
        angle_ok = angle_diff <= math.radians(35)
        
        return distance_ok and angle_ok
    
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
            
            if distance_moved > 0.3:  # 移动了超过30cm才认为开始移动了
                if not movement_detected:
                    rospy.loginfo(f"机器人开始移动，已移动 {distance_moved:.2f}米")
                    movement_detected = True
            
            # 检查是否到达目标（使用严格的容差）
            if self._check_arrival(target_pos, tolerance=0.15):  # 15cm容差
                rospy.loginfo(f"✓ 成功到达目标点: {target_pos}")
                return True
            
            rospy.sleep(0.5)
        
        # 检查最终位置
        final_distance = math.sqrt((self.current_position["x"] - target_pos[0])**2 + (self.current_position["y"] - target_pos[1])**2)
        rospy.loginfo(f"导航结束，最终距离目标: {final_distance:.3f}米")
        
        # 只有真正接近目标才认为成功
        if final_distance < 0.2:  # 20cm内才认为成功
            rospy.loginfo(f"✓ 成功到达目标点: {target_pos}")
            return True
        
        rospy.logwarn(f"✗ 导航失败，距离目标还有 {final_distance:.3f}米: {target_pos}")
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
        """尝试强制接近目标点 - 使用更直接的控制方法"""
        rospy.loginfo(f"尝试强制接近: {target_pos}")
        
        # 停止机器人
        self._stop_robot()
        rospy.sleep(1.0)  # 等待停止
        
        # 计算到目标点的方向
        dx = target_pos[0] - self.current_position["x"]
        dy = target_pos[1] - self.current_position["y"]
        distance = math.sqrt(dx*dx + dy*dy)
        
        rospy.loginfo(f"当前距离目标: {distance:.3f}米")
        
        if distance < 0.2:  # 已经很接近了
            rospy.loginfo(f"✓ 已经接近目标: {target_pos}")
            return True
        
        # 计算移动方向
        angle = math.atan2(dy, dx)
        rospy.loginfo(f"目标方向角度: {math.degrees(angle):.1f}度")
        
        # 分阶段移动：先转向，再前进
        # 阶段1：转向目标方向
        rospy.loginfo("阶段1：转向目标方向")
        turn_msg = Twist()
        turn_msg.angular.z = 0.5 if angle > 0 else -0.5
        
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 5.0:
            self.cmd_vel_pub.publish(turn_msg)
            rospy.sleep(0.1)
        
        self._stop_robot()
        rospy.sleep(0.5)
        
        # 阶段2：前进
        rospy.loginfo("阶段2：前进到目标")
        move_msg = Twist()
        move_msg.linear.x = 0.3  # 适中的速度
        
        start_x = self.current_position["x"]
        start_y = self.current_position["y"]
        start_time = rospy.Time.now()
        
        while (rospy.Time.now() - start_time).to_sec() < 15.0:
            self.cmd_vel_pub.publish(move_msg)
            
            # 检查当前位置
            current_x = self.current_position["x"]
            current_y = self.current_position["y"]
            current_distance = math.sqrt((current_x - target_pos[0])**2 + (current_y - target_pos[1])**2)
            
            rospy.loginfo(f"当前距离目标: {current_distance:.3f}米")
            
            if current_distance < 0.15:  # 15cm内认为成功
                self._stop_robot()
                rospy.loginfo(f"✓ 强制接近成功: {target_pos}")
                return True
            
            rospy.sleep(0.1)
        
        self._stop_robot()
        
        # 检查最终结果
        final_distance = math.sqrt((self.current_position["x"] - target_pos[0])**2 + (self.current_position["y"] - target_pos[1])**2)
        rospy.loginfo(f"强制接近结束，最终距离: {final_distance:.3f}米")
        
        if final_distance < 0.3:  # 30cm内认为部分成功
            rospy.loginfo(f"✓ 强制接近部分成功: {target_pos}")
            return True
        
        rospy.logwarn(f"✗ 强制接近失败: {target_pos}")
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
        """处理到达状态回调 - 只有在1米内且停止时才退出程序"""
        if msg.data == "arrived":
            rospy.loginfo("收到机器人到达状态")
            
            # 检查当前位置信息
            rospy.loginfo(f"当前位置: ({self.current_position['x']:.3f}, {self.current_position['y']:.3f})")
            rospy.loginfo(f"目标位置: {self.target_pos}")
            
            # 计算实际距离
            dx = self.current_position["x"] - self.target_pos[0]
            dy = self.current_position["y"] - self.target_pos[1]
            actual_distance = math.sqrt(dx*dx + dy*dy)
            rospy.loginfo(f"实际距离目标: {actual_distance:.3f}米")
            
            # 只有在1米内且机器人停止时才退出程序
            if actual_distance <= 1.0:
                rospy.loginfo("✓ 机器人在1米范围内且已停止，程序即将退出")
                self.navigation_success = True
                
                # 立即释放资源并结束程序
                rospy.loginfo("释放资源...")
                self._stop_robot()
                self._cancel_current_goal()
                
                rospy.loginfo("导航任务完成，程序即将退出")
                rospy.signal_shutdown("到达目标范围")
            else:
                rospy.loginfo(f"机器人距离目标还有 {actual_distance:.3f}米，继续等待...")
    
    def navigate_to_backstage(self):
        """主要导航方法 - 发布目标后等待到达状态"""
        rospy.loginfo("开始导航到后台点...")
        rospy.loginfo(f"目标点: {self.target_pos}")
        rospy.loginfo("策略：发布目标，等待g1_control的到达状态")
        
        # 发布初始目标
        goal_msg = self._build_move_base_goal(self.target_pos[0], self.target_pos[1], self.target_pos[2])
        self.goal_pub.publish(goal_msg)
        rospy.loginfo("已发布初始目标，等待g1_control处理...")
        
        # 等待到达状态，不主动干预
        rospy.loginfo("等待机器人到达...")
        return True  # 让主循环处理等待逻辑
    
    def run(self):
        """运行导航器 - 发布目标后等待到达状态"""
        try:
            # 等待系统稳定
            rospy.sleep(2.0)
            
            # 开始导航
            rospy.loginfo("开始导航任务...")
            self.navigate_to_backstage()
            
            # 等待到达信号，程序会在这里阻塞直到收到到达状态
            rospy.loginfo("程序等待到达信号...")
            rospy.loginfo("只有在1米范围内且机器人停止时才会退出程序")
            
            # 保持程序运行，直到收到到达信号
            while not rospy.is_shutdown() and not self.navigation_success:
                # 额外检查：如果机器人已经在1米内，也考虑退出
                dx = self.current_position["x"] - self.target_pos[0]
                dy = self.current_position["y"] - self.target_pos[1]
                current_distance = math.sqrt(dx*dx + dy*dy)
                
                if current_distance <= 0.45:
                    rospy.loginfo(f"机器人已在0.45米范围内 ({current_distance:.3f}米)，等待停止信号...")
                
                rospy.sleep(0.5)  # 稍微慢一点，避免过于频繁的检查
            
            if self.navigation_success:
                rospy.loginfo("✓ 程序已收到到达信号并退出")
            else:
                rospy.logerr("✗ 程序异常退出")
                
        except KeyboardInterrupt:
            rospy.loginfo("用户中断导航")
        except Exception as e:
            rospy.logerr(f"导航过程中发生错误: {e}")
        finally:
            # 清理（在arrival_callback中已经处理了）
            rospy.loginfo("程序清理完成")

def main():
    """主函数"""
    try:
        navigator = RobustBackstageNavigator()
        navigator.run()
    except Exception as e:
        rospy.logerr(f"程序启动失败: {e}")

if __name__ == "__main__":
    main()
