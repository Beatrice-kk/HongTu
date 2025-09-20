#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import os
import threading
import tf.transformations as tft
import argparse
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan, GetPlanRequest

class NavWaypointPlayer:
    def __init__(self, backstage_pos, stage_entry_pos, dance_type, dance_choreography, threshold=0.5):
        """
        使用航点信息初始化导航器。
        """
        # 检查舞蹈类型是否存在于编排中
        if dance_type not in dance_choreography:
            rospy.logerr(f"舞蹈类型 '{dance_type}' 未定义编排，将使用默认编排")
            self.dance_sequence = list(dance_choreography.values())[0]
        else:
            self.dance_sequence = dance_choreography[dance_type]
        
        # 提取舞蹈点位和等待时间
        dance_waypoints = [pos for pos, _ in self.dance_sequence]
        self.wait_times = [0]
        self.wait_times.append(self.dance_sequence[0][1])
        for i in range(1, len(self.dance_sequence)):
            self.wait_times.append(self.dance_sequence[i][1])
        self.wait_times.append(0)
        
        # 构建完整的航点序列
        self.waypoints = [backstage_pos, stage_entry_pos] + dance_waypoints + [stage_entry_pos, backstage_pos]
        self.threshold = threshold
        self.current_waypoint_index = 0
        self.reached_final = False

        # 分段导航相关
        self.sub_waypoint_index = None  # None 表示未分段

        # 确定在哪些点位执行舞蹈
        self.dance_at_waypoint = [False]
        self.dance_at_waypoint.append(True)
        self.dance_at_waypoint.extend([True] * len(dance_waypoints))
        self.dance_at_waypoint.append(False)
        
        self.dance_type = dance_type
        self.dance_in_progress = False

        # 规划相关
        self.make_plan = None
        try:
            rospy.wait_for_service('/move_base/make_plan', timeout=2.0)
            self.make_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
            rospy.loginfo("连接 make_plan 成功")
        except Exception:
            rospy.logwarn("make_plan 服务不可用，将使用基于距离的退避策略")

        # 进度/卡滞检测
        self._last_progress_check_time = rospy.Time.now()
        self._last_progress_dist = float('inf')
        self._stuck_timeout = rospy.Duration(10.0)
        self._progress_eps = 0.2
        
        # 发布导航目标
        self.goal_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=1)
        self.feedback_sub = rospy.Subscriber("/move_base/feedback", MoveBaseActionFeedback, self.feedback_callback)
        
        # 舞蹈服务客户端
        rospy.loginfo("等待舞蹈服务...")
        self.dance_direction_pub = rospy.Publisher('dance_direction', String, queue_size=1)
        try:
            rospy.wait_for_service('play_dance', timeout=5.0)
            self.play_dance_service = rospy.ServiceProxy('play_dance', Trigger)
            rospy.loginfo("舞蹈服务已连接")
        except rospy.ROSException:
            rospy.logwarn("舞蹈服务不可用，将不执行舞蹈动作")
            self.play_dance_service = None
        
        rospy.sleep(1.0)
        rospy.loginfo(f"开始表演，舞蹈类型: {self.dance_type}")
        self.navigate_to_current_waypoint()

    def _build_move_base_goal(self, x, y, theta_deg):
        """
        构造MoveBaseActionGoal消息用于发布到move_base/goal
        """
        goal = MoveBaseActionGoal()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.goal_id.stamp = rospy.Time.now()
        goal.goal_id.id = ""
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

    def _build_ps(self, x, y, theta_deg):
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = rospy.Time.now()
        ps.pose.position.x = x
        ps.pose.position.y = y
        q = tft.quaternion_from_euler(0, 0, math.radians(theta_deg))
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]
        return ps

    def _split_path_linear(self, start_xy, end_xytheta, num_segments=3):
        sx, sy = start_xy
        ex, ey, eth = end_xytheta
        points = []
        for i in range(1, num_segments):
            ratio = i / num_segments
            x = sx + (ex - sx) * ratio
            y = sy + (ey - sy) * ratio
            points.append((x, y, eth))
        points.append((ex, ey, eth))
        return points

    def _try_find_reachable_nearby(self, target_xytheta, current_xy):
        tx, ty, tt = target_xytheta
        cx, cy = current_xy
        radii = [0.3, 0.6, 1.0]
        angles = [0, math.pi/3, 2*math.pi/3, math.pi, -2*math.pi/3, -math.pi/3]
        candidates = [(tx, ty, tt)]
        for r in radii:
            for a in angles:
                candidates.append((tx + r*math.cos(a), ty + r*math.sin(a), tt))

        if self.make_plan is not None:
            try:
                start = self._build_ps(cx, cy, 0.0)
                best = None
                best_len = None
                for (x,y,th) in candidates:
                    goal = self._build_ps(x, y, th)
                    req = GetPlanRequest()
                    req.start = start
                    req.goal = goal
                    req.tolerance = 0.2
                    resp = self.make_plan(req)
                    if resp.plan and len(resp.plan.poses) > 0:
                        plen = len(resp.plan.poses)
                        if best is None or plen < best_len:
                            best = (x, y, th)
                            best_len = plen
                if best is not None:
                    return best
                else:
                    rospy.logwarn("所有候选点都不可达，尝试分段导航")
                    split_points = self._split_path_linear(current_xy, target_xytheta, num_segments=3)
                    return split_points
            except Exception as e:
                rospy.logwarn("make_plan 调用失败: %s", str(e))
                split_points = self._split_path_linear(current_xy, target_xytheta, num_segments=3)
                return split_points

        best = None
        best_d = None
        for (x,y,th) in candidates:
            d = math.hypot(x-cx, y-cy)
            if best is None or d < best_d:
                best = (x,y,th)
                best_d = d
        if best is not None:
            return best
        split_points = self._split_path_linear(current_xy, target_xytheta, num_segments=3)
        return split_points

    def navigate_to_current_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints):
            rospy.loginfo("[完成] 所有航点已完成!")
            self.reached_final = True
            return

        wp = self.waypoints[self.current_waypoint_index]
        # 分段点列支持
        if isinstance(wp, list):
            sub_index = self.sub_waypoint_index if self.sub_waypoint_index is not None else 0
            x, y, theta = wp[sub_index]
            location_desc = f"[分段航点 {sub_index+1}/{len(wp)} in {self.current_waypoint_index+1}/{len(self.waypoints)}]"
        else:
            x, y, theta = wp
            location_desc = ""
            if self.current_waypoint_index == 0:
                location_desc = "[后台起点]"
            elif self.current_waypoint_index == 1:
                location_desc = "[舞台入口]"
            elif self.current_waypoint_index == len(self.waypoints) - 1:
                location_desc = "[返回后台]"
            else:
                location_desc = f"[舞蹈点位 {self.current_waypoint_index-1}]"

        wait_time = self.wait_times[self.current_waypoint_index]
        rospy.loginfo(f"[导航目标 {self.current_waypoint_index+1}/{len(self.waypoints)}] {location_desc} x={x}, y={y}, θ={theta} (等待时间: {wait_time}秒)")
        # 发布MoveBaseActionGoal
        self.goal_pub.publish(self._build_move_base_goal(x, y, theta))
        self._last_progress_check_time = rospy.Time.now()
        self._last_progress_dist = float('inf')

    def perform_dance(self):
        if self.play_dance_service is None:
            rospy.logwarn("舞蹈服务不可用，跳过舞蹈")
            return True
        try:
            dance_direction = self.dance_type
            rospy.loginfo(f"开始执行舞蹈: {dance_direction}")
            self.dance_direction_pub.publish(String(dance_direction))
            rospy.sleep(0.5)
            response = self.play_dance_service()
            if response.success:
                rospy.loginfo(f"舞蹈执行成功: {response.message}")
                return True
            else:
                rospy.logwarn(f"舞蹈执行失败: {response.message}")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"舞蹈服务调用失败: {e}")
            return False

    def feedback_callback(self, msg):
        if self.reached_final or self.dance_in_progress:
            return

        # 支持分段点列
        wp = self.waypoints[self.current_waypoint_index]
        if isinstance(wp, list):
            sub_waypoints = wp
            sub_index = self.sub_waypoint_index if self.sub_waypoint_index is not None else 0
            x, y, _ = sub_waypoints[sub_index]
        else:
            x, y, _ = wp
            sub_waypoints = None

        current_pose = msg.feedback.base_position.pose
        dx = current_pose.position.x - x
        dy = current_pose.position.y - y
        dist = math.hypot(dx, dy)

        rospy.loginfo_throttle(2, f"[当前位置] ({current_pose.position.x:.2f}, {current_pose.position.y:.2f}) -> 距离目标 {dist:.2f} m")

        now = rospy.Time.now()
        # 卡滞处理
        if now - self._last_progress_check_time > self._stuck_timeout and not self.reached_final and not self.dance_in_progress:
            progressed = False
            if self._last_progress_dist != float('inf'):
                progressed = (self._last_progress_dist - dist) > self._progress_eps
            self._last_progress_check_time = now
            self._last_progress_dist = dist

            if not progressed:
                rospy.logwarn("[规划] 目标可能不可达/无进展，尝试在附近/直线上分段搜寻可达点...")
                cur_xy = (current_pose.position.x, current_pose.position.y)
                result = self._try_find_reachable_nearby(
                    wp if not sub_waypoints else sub_waypoints[sub_index],
                    cur_xy
                )
                if isinstance(result, list):
                    self.waypoints[self.current_waypoint_index] = result
                    self.sub_waypoint_index = 0
                    nx, ny, nth = result[0]
                    rospy.loginfo(f"[分段导航] 使用分段目标: x={nx:.2f}, y={ny:.2f}, θ={nth:.1f}")
                    # 发布MoveBaseActionGoal分段点
                    self.goal_pub.publish(self._build_move_base_goal(nx, ny, nth))
                    self._last_progress_check_time = rospy.Time.now()
                    self._last_progress_dist = float('inf')
                    return
                elif result is not None:
                    if sub_waypoints:
                        self.waypoints[self.current_waypoint_index][sub_index] = result
                    else:
                        self.waypoints[self.current_waypoint_index] = result
                    nx, ny, nth = result
                    rospy.loginfo(f"[规划] 使用替代目标: x={nx:.2f}, y={ny:.2f}, θ={nth:.1f}")
                    self.navigate_to_current_waypoint()
                    self._last_progress_check_time = rospy.Time.now()
                    self._last_progress_dist = float('inf')
                    return
                else:
                    rospy.logwarn("[规划] 未找到替代目标，继续保持当前目标")

        if dist < self._last_progress_dist:
            self._last_progress_dist = dist

        if dist <= self.threshold:
            self.dance_in_progress = True
            if sub_waypoints:
                sub_index = self.sub_waypoint_index if self.sub_waypoint_index is not None else 0
                sub_index += 1
                if sub_index < len(sub_waypoints):
                    self.sub_waypoint_index = sub_index
                    nx, ny, nth = sub_waypoints[sub_index]
                    rospy.loginfo(f"[分段到达] 前往下一个分段点: x={nx}, y={ny}, θ={nth}")
                    # 发布MoveBaseActionGoal分段点
                    self.goal_pub.publish(self._build_move_base_goal(nx, ny, nth))
                    self.dance_in_progress = False
                    return
                else:
                    self.sub_waypoint_index = None
                    self.current_waypoint_index += 1
            else:
                self.current_waypoint_index += 1

            # 判断是否需要在当前点位执行舞蹈
            should_dance = self.dance_at_waypoint[self.current_waypoint_index - 1 if not sub_waypoints else self.current_waypoint_index]
            dance_thread = threading.Thread(target=self._execute_dance_and_continue, args=(should_dance,))
            dance_thread.daemon = True
            dance_thread.start()

    def _execute_dance_and_continue(self, should_dance):
        try:
            if should_dance:
                rospy.loginfo("/* 开始执行指定舞蹈动作... */")
                self.perform_dance()
                rospy.loginfo("/* 舞蹈动作执行完毕 */")
            wait_time = self.wait_times[self.current_waypoint_index - 1]
            if self.current_waypoint_index < len(self.waypoints):
                if wait_time > 0:
                    rospy.loginfo(f"等待 {wait_time} 秒后前往下一个航点...")
                    rospy.sleep(wait_time)
                else:
                    rospy.loginfo("立即前往下一个航点...")
                self.navigate_to_current_waypoint()
            else:
                rospy.loginfo("[完成] 所有航点已完成!")
                self.reached_final = True
        except Exception as e:
            rospy.logerr(f"舞蹈执行线程错误: {e}")
        finally:
            self.dance_in_progress = False

if __name__ == "__main__":
    rospy.init_node("nav_waypoints_player")
    parser = argparse.ArgumentParser(description='导航舞蹈表演控制器')
    parser.add_argument('--dance', type=str, default='A', 
                      choices=['A', 'B', 'Up', 'Down', 'Left', 'Right', 'X', 'Y'],
                      help='指定要执行的舞蹈类型')
    args, unknown = parser.parse_known_args()
    backstage_pos =(0,0,0)
    stage_entry_pos = (-1.86,0.890,180)

    dance_choreography = {
        'A': [
            ((-3.35, 3.0, 151), 3.0),
            ((-3, 3.0, 138), 5.0),
            ((-2.7, 3.0, 114), 2.0),
        ],
        'B': [
            ((4.18, 1.15, -159), 2.0),
            ((-3.5, 0.5, 90), 4.0),
            ((2.3, 1.7, -45), 3.0),
            ((-1.5, -1.2, 180), 2.5),
        ],
        'Up': [
            ((4.18, 1.15, -159), 2.5),
            ((3.0, 3.0, 0), 3.0),
            ((-2.0, 2.0, 45), 4.0),
        ],
        'Down': [
            ((4.18, 1.15, -159), 1.5),
            ((0.0, 0.0, -90), 3.5),
            ((2.5, -2.0, 135), 2.0),
            ((-2.5, -1.0, -135), 4.5),
        ],
        'Left': [
            ((4.18, 1.15, -159), 2.0),
            ((-4.0, 1.0, 90), 3.0),
            ((-2.0, -1.0, -90), 2.5),
        ],
        'Right': [
            ((4.18, 1.15, -159), 2.0),
            ((3.0, -1.0, -90), 3.0),
            ((1.0, 2.0, 0), 2.5),
        ],
        'X': [
            ((4.18, 1.15, -159), 3.0),
            ((-3.0, 3.0, 135), 4.0),
            ((3.0, -3.0, -45), 3.5),
            ((-3.0, -3.0, -135), 2.5),
            ((3.0, 3.0, 45), 3.0),
        ],
        'Y': [
            ((4.18, 1.15, -159), 2.5),
            ((0.0, 3.0, 90), 3.0),
            ((-2.0, 0.0, 180), 4.0),
            ((2.0, 0.0, 0), 3.5),
        ]
    }
    threshold = 0.5
    node = NavWaypointPlayer(
        backstage_pos=backstage_pos,
        stage_entry_pos=stage_entry_pos,
        dance_type=args.dance,
        dance_choreography=dance_choreography,
        threshold=threshold
    )
    rospy.loginfo(f"表演开始，使用舞蹈类型: {args.dance}")
    rospy.spin()