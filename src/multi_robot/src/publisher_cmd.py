#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import Twist

def movimiento():
  vel = Twist()
  vel.linear.x = 0.5
  vel.angular.z = 0.3
  pub.publish(vel)

topic = str(sys.argv[1]) + '/cmd_vel'
rospy.init_node('publisher_cmd', anonymous=True)
pub = rospy.Publisher(topic, Twist, queue_size =5)
rate = rospy.Rate(10) # 0.1 segundo

while(1):
  movimiento()
  rate.sleep()
