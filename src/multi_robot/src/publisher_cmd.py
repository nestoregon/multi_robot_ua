#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


"""
Definir un movimiento
"""
class square_movement():
  def __init__(self, x=0, z=0):
    """Initialize node"""
    rospy.init_node('publisher_cmd', anonymous= True)
    self.odom = Odometry()
    # self.pose = Pose()
    self.vel = Twist()
    self.vel.linear.x = x
    self.vel.angular.z = z
    self.pub_topic_name = str(sys.argv[1]) + '/cmd_vel'
    self.sub_topic_name = str(sys.argv[1]) + '/odom'
    
    """Create publishers y subscribers"""
    self.pub = rospy.Publisher(self.pub_topic_name, Twist, queue_size = 5)
    self.sub = rospy.Subscriber(self.sub_topic_name, Odometry, self.callback)
    self.rate = rospy.Rate(5) # 0.2 segundos

  def callback(self, msg):
    # do something
    self.odom.pose.pose.orientation = msg.pose.pose.orientation

  def avanzar(self):
    print("hola")

  def giro(self):
    self.pub.publish(self.vel)

  def move(self):

    self.vel.angular.z = 0.2
    while 1:
      self.giro()
      print(self.odom.pose.pose.orientation)
      print("----------------")
      self.rate.sleep()
    

def main():
  movement = square_movement()
  movement.move()
  rospy.spin()

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    print("Shutting down publisher")
    pass