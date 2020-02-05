#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def movimiento():
  vel = Twist()
  vel.linear.x = 0.5
  vel.angular.z = 0.3
  pub1.publish(vel)


rospy.init_node('publisher_cmd', anonymous=True)
pub1 = rospy.Publisher('robot2/cmd_vel', Twist, queue_size =5)
rate = rospy.Rate(10) # 0.1 segundo

while(1):
  movimiento()
  rate.sleep()