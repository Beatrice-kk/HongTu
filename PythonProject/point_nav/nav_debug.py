#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import tf.transformations as tft
import argparse
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionFeedback
from nav_msgs.srv import GetPlan, GetPlanRequest

class NavWaypointDebugger:
    def __init__(self, waypoints, threshold=0.5):
        """
        初始化导航调试器，仅负责点位移动，无舞蹈相关功能。
        
        参数:
            waypoints: 航点列表，每个为 (x, y, theta) 元组
            threshold: 认为到达航点的距离阈值(米)
        """
        self.waypoints = waypoints
        self.threshold = threshold
        self.current_waypoint_index = 0
        self.reached_final = False
        
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
        
        # 发布导航目标（PoseStamped）
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        # 订阅 /move_base/feedback 获取当前位姿
        self.feedback_sub = rospy.Subscriber("/move_base/feedback", MoveBaseActionFeedback, self.feedback_callback)
        
        rospy.sleep(1.0)  # 等待topic连接
        rospy.loginfo(f"开始导航调试，共 {len(self.waypoints)} 个航点")
        self.navigate_to_current_waypoint()

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

    def navigate_to_current_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints):
            rospy.loginfo("[完成] 所有航点已完成!")
            self.reached_final = True
            return

        wp = self.waypoints[self.current_waypoint_index]
        # 分段点列支持
        if isinstance(wp, list):
            sub_index = getattr(self, 'sub_waypoint_index', 0)
            x, y, theta = wp[sub_index]
            location_desc = f"[分段航点 {sub_index+1}/{len(wp)} in {self.current_waypoint_index+1}/{len(self.waypoints)}]"
        else:
            x, y, theta = wp
            location_desc = f"[航点 {self.current_waypoint_index+1}/{len(self.waypoints)}]"
        
        goal_ps = self._build_ps(x, y, theta)
        rospy.loginfo(f"{location_desc} x={x}, y={y}, θ={theta}")
        self.goal_pub.publish(goal_ps)
        self._last_progress_check_time = rospy.Time.now()
        self._last_progress_dist = float('inf')

    def _split_path_linear(self, start_xy, end_xytheta, num_segments=3):
        """
        将当前位置到目标点连线等分为num_segments段，返回所有分割点
        """
        sx, sy = start_xy
        ex, ey, eth = end_xytheta
        points = []
        for i in range(1, num_segments):
            ratio = i / num_segments
            x = sx + (ex - sx) * ratio
            y = sy + (ey - sy) * ratio
            points.append((x, y, eth))
        points.append((ex, ey, eth))  # 最后是目标点
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
                    return split_points  # 返回多个点
            except Exception as e:
                rospy.logwarn("make_plan 调用失败: %s", str(e))
                split_points = self._split_path_linear(current_xy, target_xytheta, num_segments=3)
                return split_points

        # 没有make_plan服务时，默认选距离最近（其实就是目标点），如果也不可达则分段
        best = None
        best_d = None
        for (x,y,th) in candidates:
            d = math.hypot(x-cx, y-cy)
            if best is None or d < best_d:
                best = (x,y,th)
                best_d = d
        if best is not None:
            return best
        # 不可达，分段
        split_points = self._split_path_linear(current_xy, target_xytheta, num_segments=3)
        return split_points

    def feedback_callback(self, msg):
        if self.reached_final:
            return

        current_pose = msg.feedback.base_position.pose

        # 支持分段点列
        wp = self.waypoints[self.current_waypoint_index]
        if isinstance(wp, list):  # 分段点列
            sub_waypoints = wp
            sub_index = getattr(self, 'sub_waypoint_index', 0)
            x, y, _ = sub_waypoints[sub_index]
        else:
            x, y, _ = wp
            sub_waypoints = None

        dx = current_pose.position.x - x
        dy = current_pose.position.y - y
        dist = math.hypot(dx, dy)

        rospy.loginfo_throttle(2, f"[当前位置] ({current_pose.position.x:.2f}, {current_pose.position.y:.2f}) -> 距离目标 {dist:.2f} m")

        now = rospy.Time.now()
        if now - self._last_progress_check_time > self._stuck_timeout and not self.reached_final:
            progressed = False
            if self._last_progress_dist != float('inf'):
                progressed = (self._last_progress_dist - dist) > self._progress_eps
            self._last_progress_check_time = now
            self._last_progress_dist = dist

            if not progressed:
                rospy.logwarn("[规划] 目标可能不可达/无进展，尝试在附近/直线上分段搜寻可达点...")
                cur_xy = (current_pose.position.x, current_pose.position.y)
                result = self._try_find_reachable_nearby(
                    self.waypoints[self.current_waypoint_index] if not sub_waypoints else sub_waypoints[sub_index],
                    cur_xy
                )
                if isinstance(result, list):  # 分段点列
                    self.waypoints[self.current_waypoint_index] = result
                    self.sub_waypoint_index = 0
                    nx, ny, nth = result[0]
                    rospy.loginfo(f"[分段导航] 使用分段目标: x={nx:.2f}, y={ny:.2f}, θ={nth:.1f}")
                    self.goal_pub.publish(self._build_ps(nx, ny, nth))
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
            if sub_waypoints:
                sub_index = getattr(self, 'sub_waypoint_index', 0)
                sub_index += 1
                if sub_index < len(sub_waypoints):
                    self.sub_waypoint_index = sub_index
                    nx, ny, nth = sub_waypoints[sub_index]
                    rospy.loginfo(f"[分段到达] 前往下一个分段点: x={nx}, y={ny}, θ={nth}")
                    self.goal_pub.publish(self._build_ps(nx, ny, nth))
                    return
                else:
                    # 分段结束，进入下一个主航点
                    del self.sub_waypoint_index
                    self.current_waypoint_index += 1
            else:
                self.current_waypoint_index += 1

            if self.current_waypoint_index < len(self.waypoints):
                rospy.loginfo("前往下一个航点...")
                self.navigate_to_current_waypoint()
            else:
                rospy.loginfo("[完成] 所有航点已完成!")
                self.reached_final = True

if __name__ == "__main__":
    rospy.init_node("nav_waypoints_debugger")
    parser = argparse.ArgumentParser(description='导航路径调试器')
    parser.add_argument('--sequence', type=str, default='A', 
                        choices=['A', 'B', 'Up', 'Down', 'Left', 'Right', 'X', 'Y'],
                        help='指定要测试的点位序列类型')
    args, unknown = parser.parse_known_args()

    backstage_pos =(0,0,0)
    stage_entry_pos = (-1.86,0.890,180)

    waypoint_sequences = {
        'A': [
            backstage_pos,
            stage_entry_pos,
            (-3.35, 3.0, 151),
            (-3, 3.0, 138),
            (-2.7, 3.0, 114),
            stage_entry_pos,
            backstage_pos,
        ],
        'B': [
            backstage_pos,
            stage_entry_pos,
            (4.18, 1.15, -159),
            (-3.5, 0.5, 90),
            (2.3, 1.7, -45),
            (-1.5, -1.2, 180),
            backstage_pos,
        ],
        'Up': [
            backstage_pos,
            stage_entry_pos,
            (4.18, 1.15, -159),
            (3.0, 3.0, 0),
            (-2.0, 2.0, 45),
            backstage_pos,
        ],
        'Down': [
            backstage_pos,
            stage_entry_pos,
            (4.18, 1.15, -159),
            (0.0, 0.0, -90),
            (2.5, -2.0, 135),
            (-2.5, -1.0, -135),
            backstage_pos,
        ],
        'Left': [
            backstage_pos,
            stage_entry_pos,
            (4.18, 1.15, -159),
            (-4.0, 1.0, 90),
            (-2.0, -1.0, -90),
            backstage_pos,
        ],
        'Right': [
            backstage_pos,
            stage_entry_pos,
            (4.18, 1.15, -159),
            (3.0, -1.0, -90),
            (1.0, 2.0, 0),
            backstage_pos,
        ],
        'X': [
            backstage_pos,
            stage_entry_pos,
            (4.18, 1.15, -159),
            (-3.0, 3.0, 135),
            (3.0, -3.0, -45),
            (-3.0, -3.0, -135),
            (3.0, 3.0, 45),
            backstage_pos,
        ],
        'Y': [
            backstage_pos,
            stage_entry_pos,
            (4.18, 1.15, -159),
            (0.0, 3.0, 90),
            (-2.0, 0.0, 180),
            (2.0, 0.0, 0),
            backstage_pos,
        ]
    }

    threshold = 0.5

    waypoints = waypoint_sequences.get(args.sequence, waypoint_sequences['A'])
    node = NavWaypointDebugger(
        waypoints=waypoints,
        threshold=threshold
    )
    rospy.loginfo(f"导航调试开始，点位序列类型: {args.sequence}")
    rospy.spin()