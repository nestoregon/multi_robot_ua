#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry


"""
Square movement class
"""
class square_movement():

  """Constructor"""
  def __init__(self, x=0, z=0):
    # initialize node
    # declare self variables
    self.odom = Odometry()
    self.vel = Twist()
    self.vel.linear.x = x
    self.vel.angular.z = z
    self.pub_topic_name = str(sys.argv[1]) + '/cmd_vel'
    self.sub_topic_name = str(sys.argv[1]) + '/odom'
    self.turns_array = [0.7071, 1, 0.7071, 0, -0.7071, -1, -0.7071, 0] # dos vueltas
    self.turn = self.turns_array[0]
    # publishers
    self.pub = rospy.Publisher(self.pub_topic_name, Twist, queue_size = 5)
    # subscribers
    self.sub = rospy.Subscriber(self.sub_topic_name, Odometry, self.callback_odom)
    # rate
    self.rate = rospy.Rate(100) # 0.01 segundos
    self.state = "paused"
    self.time = 0


  """callback function of odometry"""
  def callback_odom(self, msg):
    # save odometry published value
    self.odom.pose.pose.orientation = msg.pose.pose.orientation 

  def move_forward(self):
    print("Going on a straight line")
    self.vel.linear.x = 0.2
    self.vel.angular.z = 0.0
    self.pub.publish(self.vel)

  def turn_left(self):
    print("Turning")
    self.vel.linear.x = 0.0
    self.vel.angular.z = 0.2
    self.pub.publish(self.vel)


  def move(self):
    self.state = "move_forward"
    while not rospy.is_shutdown():
      
      for item in self.turns_array:
        pose = self.odom.pose.pose.orientation.z
        print("waiting to reach", item, "Pose is", pose)

        # move forward state
        if self.state == "move_forward":
          if self.time == 100: # wait for 1 second
            self.state = "turn_left"
            self.turn_left()
            self.time = 0
          self.time += 1

        # turn left state
        elif self.state == "turn_left":
          if not(pose-0.05 < item and pose +0.05 > item):
            self.state = "move_forward"
            self.move_forward()

        self.rate.sleep()
        
    

def main():
  movement = square_movement()
  movement.move()
  rospy.spin()

if __name__ == "__main__":
  # initialize node
  rospy.init_node('publisher_cmd', anonymous= True)
  try:
    main()
  except rospy.ROSInterruptException:
    print("Shutting down publisher")
    pass