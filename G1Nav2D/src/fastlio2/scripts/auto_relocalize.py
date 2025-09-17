#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
import random
import time
import subprocess
import re

class AutoRelocalize:
    def __init__(self):
        rospy.init_node('auto_relocalize', anonymous=True)
        
        # 发布2D姿态估计的话题
        self.pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        
        # 清除代价地图服务（如果需要的话）
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        
        # 设置地图边界 - 根据您的地图调整这些值
        self.x_min, self.x_max = -5.0, 5.0
        self.y_min, self.y_max = -5.0, 5.0
        
        # 配置参数
        self.attempts_per_pose = 3  # 每个位置尝试的次数
        self.max_total_attempts = 100  # 最大尝试次数
        self.wait_time = 1.0  # 每次尝试之间的等待时间
        
        # 默认成功的ICP分数阈值（可以根据您的系统调整）
        self.success_threshold = 0.03  # ICP分数低于此值表示成功
        
        # 从launch文件或参数服务器获取配置
        self.grid_search = rospy.get_param('~grid_search', False)  # 是否使用网格搜索策略
        self.grid_size = rospy.get_param('~grid_size', 5)  # 网格大小
        
        # 记录重定位的历史
        self.attempts = 0
        self.successful = False
        
    def create_pose_msg(self, x, y, yaw):
        """创建一个2D姿态估计消息"""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = rospy.Time.now()
        
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0
        
        # 四元数表示方向
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose_msg.pose.pose.orientation.x = quaternion[0]
        pose_msg.pose.pose.orientation.y = quaternion[1]
        pose_msg.pose.pose.orientation.z = quaternion[2]
        pose_msg.pose.pose.orientation.w = quaternion[3]
        
        # 设置协方差（这个值可能需要调整）
        pose_msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        
        return pose_msg
    
    def check_icp_success(self):
        """检查最近的日志以确定ICP是否成功"""
        try:
            # 运行grep命令获取最近的ICP日志
            cmd = "tail -n 50 ~/.ros/log/latest/stdout.log | grep -A 5 'ICP'"
            result = subprocess.check_output(cmd, shell=True, text=True)
            
            # 检查成功消息
            if "Alignment SUCCESS" in result:
                # 提取分数以进行更精确的判断
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
    
    def grid_search_poses(self):
        """生成网格搜索的姿态列表"""
        poses = []
        x_step = (self.x_max - self.x_min) / (self.grid_size - 1)
        y_step = (self.y_max - self.y_min) / (self.grid_size - 1)
        
        for i in range(self.grid_size):
            x = self.x_min + i * x_step
            for j in range(self.grid_size):
                y = self.y_min + j * y_step
                # 每个位置尝试4个不同的方向
                for yaw in [0, 1.57, 3.14, -1.57]:  # 0, 90, 180, -90度
                    poses.append((x, y, yaw))
        
        return poses
    
    def run(self):
        """执行自动重定位过程"""
        rospy.loginfo("开始自动重定位过程...")
        
        if self.grid_search:
            poses = self.grid_search_poses()
            rospy.loginfo(f"使用网格搜索策略，生成了{len(poses)}个位姿")
        
        total_attempts = 0
        
        while not rospy.is_shutdown() and total_attempts < self.max_total_attempts and not self.successful:
            if self.grid_search and total_attempts < len(poses):
                # 使用预先生成的网格搜索姿态
                x, y, yaw = poses[total_attempts]
            else:
                # 随机生成姿态
                x = random.uniform(self.x_min, self.x_max)
                y = random.uniform(self.y_min, self.y_max)
                yaw = random.uniform(-3.14, 3.14)
            
            rospy.loginfo(f"尝试姿态估计 #{total_attempts+1}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
            
            # 发布姿态估计
            pose_msg = self.create_pose_msg(x, y, yaw)
            self.pose_pub.publish(pose_msg)
            
            # 等待一会儿让ICP处理
            time.sleep(self.wait_time)
            
            # 清除代价地图
            try:
                self.clear_costmaps()
                rospy.loginfo("成功清除代价地图!")
            except rospy.ServiceException as e:
                rospy.logwarn(f"清除代价地图失败: {e}")
            
            # 再等一会儿看结果
            time.sleep(self.wait_time)
            
            # 检查是否重定位成功
            if self.check_icp_success():
                self.successful = True
                rospy.loginfo(f"重定位成功! 位置: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
                break
            
            total_attempts += 1
            
        if self.successful:
            rospy.loginfo(f"自动重定位成功，共尝试 {total_attempts} 次")
        else:
            rospy.logwarn(f"达到最大尝试次数 ({self.max_total_attempts})，重定位失败")

if __name__ == '__main__':
    try:
        relocalize = AutoRelocalize()
        relocalize.run()
    except rospy.ROSInterruptException:
        pass