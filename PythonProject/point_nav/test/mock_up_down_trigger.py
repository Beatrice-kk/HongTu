#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import argparse
from std_msgs.msg import String
from std_srvs.srv import Trigger


def simulate_reach_and_up_down(service_type: str, delay_before_service: float = 0.0):
    """
    Simulate "arrived at waypoint, then call Up/Down service":
    - Optionally wait some seconds (simulate arrival)
    - Call play_up or play_down service
    """
    
    if delay_before_service > 0:
        rospy.loginfo("Waiting before service call %.2f s..." % delay_before_service)
        rospy.sleep(delay_before_service)

    rospy.loginfo("Calling %s service..." % service_type)
    
    try:
        # 根据服务类型选择对应的服务
        if service_type == "Up":
            rospy.wait_for_service('play_up', timeout=5.0)
            service_proxy = rospy.ServiceProxy('play_up', Trigger)
        elif service_type == "Down":
            rospy.wait_for_service('play_down', timeout=5.0)
            service_proxy = rospy.ServiceProxy('play_down', Trigger)
        else:
            rospy.logerr(f"Unsupported service type: {service_type}")
            return False
    except rospy.ROSException:
        rospy.logwarn(f"{service_type} service not available")
        return False

    try:
        rospy.loginfo(f"Calling {service_type} service...")
        resp = service_proxy()
        if resp.success:
            rospy.loginfo(f"{service_type} service success: %s" % resp.message)
            return True
        else:
            rospy.logwarn(f"{service_type} service failed: %s" % resp.message)
            return False
    except rospy.ServiceException as e:
        rospy.logerr(f"{service_type} service call failed: %s" % str(e))
        return False


def main():
    rospy.init_node('mock_up_down_trigger')

    parser = argparse.ArgumentParser(description='Mock trigger: call Up/Down service after reaching waypoint')
    parser.add_argument('--service', type=str, default='Up',
                        choices=['Up', 'Down'],
                        help='Service type to call (Up or Down)')
    parser.add_argument('--delay', type=float, default=0.0,
                        help='Wait seconds before calling service (simulate travel time)')
    args, _ = parser.parse_known_args()

    ok = simulate_reach_and_up_down(args.service, delay_before_service=args.delay)
    if ok:
        rospy.loginfo(f"Done: {args.service} service executed successfully")
    else:
        rospy.loginfo(f"Done: {args.service} service not executed")


if __name__ == '__main__':
    main()
