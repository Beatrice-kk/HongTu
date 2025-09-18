#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import time
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String

sys.path.append("/home/unitree/unitree_sdk2_python/example/g1/high_level")

from dance_nav import DanceController
class DanceService:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("dance_service")
        
        # Initialize dance controller
        try:
            self.dance_controller = DanceController(network_interface="eth0")
            rospy.loginfo("Dance controller initialized successfully")
            
            # Create dance service
            self.play_dance_service = rospy.Service(
                'play_dance', 
                Trigger,  # You could create a custom service type with more parameters
                self.handle_play_dance
            )
            
            # Create dance direction subscriber
            self.dance_direction = "A"  # Default dance
            self.direction_sub = rospy.Subscriber(
                'dance_direction', 
                String, 
                self.direction_callback
            )
            
            rospy.loginfo("Dance service ready")
            
        except Exception as e:
            rospy.logerr(f"Failed to initialize dance service: {e}")
            rospy.signal_shutdown("Initialization failed")
    
    def direction_callback(self, msg):
        """Update the dance direction based on subscription"""
        direction = msg.data
        if direction in ['Up', 'Down', 'Left', 'Right', 'A', 'B', 'X', 'Y']:
            self.dance_direction = direction
            rospy.loginfo(f"Dance direction set to: {direction}")
        else:
            rospy.logwarn(f"Invalid dance direction: {direction}")
    
   #  def handle_play_dance(self, req):
   #      """Handle service request to play a dance"""
   #      response = TriggerResponse()
        
   #      try:
   #          rospy.loginfo(f"Playing dance: {self.dance_direction}")
   #          result = self.dance_controller.play_dance(
   #              self.dance_direction, 
   #              speed=1.0, 
   #              wait_for_completion=True
   #          )
            
   #          if result:
   #              response.success = True
   #              response.message = f"Successfully played dance: {self.dance_direction}"
   #          else:
   #              response.success = False
   #              response.message = f"Failed to play dance: {self.dance_direction}"
                
   #      except Exception as e:
   #          response.success = False
   #          response.message = f"Error playing dance: {str(e)}"
            
   #      return response


    def handle_play_dance(self, req):
      """处理播放舞蹈的服务请求"""
      response = TriggerResponse()
      
      try:
         rospy.loginfo(f"收到舞蹈请求: {self.dance_direction}")
         
         # 检查舞蹈是否存在
         available_dances = [dance['direction'] for dance in self.dance_controller.list_available_dances()]
         rospy.loginfo(f"可用舞蹈: {available_dances}")
         
         if self.dance_direction not in available_dances:
               rospy.logwarn(f"舞蹈 '{self.dance_direction}' 不存在！可用舞蹈: {available_dances}")
               response.success = False
               response.message = f"Dance '{self.dance_direction}' does not exist"
               return response
         
         # 检查舞蹈控制器状态
         rospy.loginfo(f"舞蹈控制器状态: {self.dance_controller.player.state}")
         
         # 尝试播放舞蹈
         result = self.dance_controller.play_dance(
               self.dance_direction, 
               speed=1.0, 
               wait_for_completion=True
         )
         
         if result:
               response.success = True
               response.message = f"Successfully played dance: {self.dance_direction}"
         else:
               response.success = False
               response.message = f"Failed to play dance: {self.dance_direction}"
                  
      except Exception as e:
         rospy.logerr(f"舞蹈播放错误: {str(e)}")
         response.success = False
         response.message = f"Error playing dance: {str(e)}"
         
      return response
if __name__ == "__main__":
    service = DanceService()
    rospy.spin()