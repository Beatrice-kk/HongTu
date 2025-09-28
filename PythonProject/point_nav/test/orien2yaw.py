#!/usr/bin/env python3
import math
import tf.transformations as tft

def rad2deg(rad):
    return rad * 180.0 / math.pi

def normalize360(deg):
    return deg % 360

def pose_to_waypoint(position, orientation):
    """
    position: dict with keys x, y, z
    orientation: dict with keys x, y, z, w
    return: tuple (x, y, yaw_deg)
    """
    x = position['x']
    y = position['y']
    quat = [orientation['x'], orientation['y'], orientation['z'], orientation['w']]
    
    _, _, yaw = tft.euler_from_quaternion(quat)
    yaw_deg = rad2deg(yaw)
    yaw_deg_360 = normalize360(yaw_deg)
    
    return (x, y, round(yaw_deg_360, 2))

if __name__ == "__main__":
    # 示例输入
    poses = [
        {
            'position': {'x': -0.2239246815443039, 'y': -0.22894036769866943, 'z': 0.0},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': -0.9778181870557396, 'w': 0.2094554679711815}
        },
        # 可以继续添加更多 pose
    ]

    waypoints = []
    for p in poses:
        wp = pose_to_waypoint(p['position'], p['orientation'])
        waypoints.append(wp)

    print("# 定义航点列表: (x, y, yaw_deg)")
    print("waypoints = [")
    for wp in waypoints:
        print(f"    ({wp[0]}, {wp[1]}, {wp[2]}),")
    print("]")
