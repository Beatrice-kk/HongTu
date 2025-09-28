#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import argparse
from std_msgs.msg import String
from std_srvs.srv import Trigger


def simulate_reach_and_dance(dance_type: str, delay_before_dance: float = 0.0):
    """
    Simulate "arrived at waypoint, then start dance":
    - Optionally wait some seconds (simulate arrival)
    - Publish dance_direction
    - Call play_dance service
    """
    direction_pub = rospy.Publisher('dance_direction', String, queue_size=1)

    if delay_before_dance > 0:
        rospy.loginfo("Waiting before dance %.2f s..." % delay_before_dance)
        rospy.sleep(delay_before_dance)

    rospy.loginfo("Set dance direction: %s" % dance_type)
    direction_pub.publish(String(dance_type))
    rospy.sleep(0.5)

    try:
        rospy.wait_for_service('play_dance', timeout=5.0)
        play_dance = rospy.ServiceProxy('play_dance', Trigger)
    except rospy.ROSException:
        rospy.logwarn("play_dance service not available")
        return False

    try:
        rospy.loginfo("Calling play_dance...")
        resp = play_dance()
        if resp.success:
            rospy.loginfo("Dance success: %s" % resp.message)
            return True
        else:
            rospy.logwarn("Dance failed: %s" % resp.message)
            return False
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % str(e))
        return False


def main():
    rospy.init_node('mock_dance_trigger')

    parser = argparse.ArgumentParser(description='Mock trigger: dance after reaching waypoint')
    parser.add_argument('--dance', type=str, default='A',
                        choices=['A', 'B', 'Up', 'Down', 'Left', 'Right', 'X', 'Y'],
                        help='Dance type to execute')
    parser.add_argument('--delay', type=float, default=0.0,
                        help='Wait seconds before triggering dance (simulate travel time)')
    args, _ = parser.parse_known_args()

    ok = simulate_reach_and_dance(args.dance, delay_before_dance=args.delay)
    if ok:
        rospy.loginfo("Done: dance executed")
    else:
        rospy.loginfo("Done: dance not executed")


if __name__ == '__main__':
    main()


