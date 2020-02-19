#!/usr/bin/env python
"""Setting interpreter"""

import sys
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class SquareMovement():
  """Square Movement class"""
  def __init__(self, robot_name, x=0, z=0):
    # initialize node
    # declare self variables
    self.robot_name = robot_name
    self.odom = Odometry()
    self.vel = Twist()
    self.vel.linear.x = x
    self.vel.angular.z = z
    self.pub_topic_name = self.robot_name + '/cmd_vel'
    self.sub_topic_name = self.robot_name + '/odom'
    self.turn_array = [0.7071, 1, 0.7071, 0, -0.7071, -1, -0.7071, 0] # there are 8 states
    self.turn_state = 0
    self.turn_num = self.turn_array[self.turn_state]
    # publishers
    self.pub = rospy.Publisher(self.pub_topic_name, Twist, queue_size = 5)
    # subscribers
    self.sub = rospy.Subscriber(self.sub_topic_name, Odometry, self.callback_odom)
    # rate
    self.rate = rospy.Rate(100) # 0.01 segundos
    self.state = "move_forward"
    self.time = 0


  def callback_odom(self, msg):
    """callback function of odometry"""
    # save odometry published value
    self.odom.pose.pose.orientation = msg.pose.pose.orientation

  def move_forward(self):
    """Move in a straight line"""
    self.vel.linear.x = 0.2
    self.vel.angular.z = 0.0
    self.pub.publish(self.vel)

  def turn_left(self):
    """Turn left 90 degrees"""
    self.vel.linear.x = 0.0
    self.vel.angular.z = 0.3
    self.pub.publish(self.vel)

  def define_turn_number(self):
    """Define the z value for a 90 degree turn"""
    self.turn_num = self.turn_array[self.turn_state]

  def move(self):
    """Funcion to perform the square movement alternating turns and moving forward"""
    self.state = "turn_left"
    self.move_forward()

    while not rospy.is_shutdown():
      # define variables
      self.define_turn_number()
      pose = self.odom.pose.pose.orientation.z
      print(self.state, self.turn_state, self.turn_num, "Pose is", pose)

      # depending on the state the robot will do different things
      # for instance: if the state is equal to "move_forward" the
      # robot will complete such action

      # move forward
      if self.state == "move_forward":
        if self.time == 400: # 4 seconds going straight
          self.state = "turn_left"
          self.time = 0
        else:
          self.time += 1
          self.move_forward()

      # turn left
      elif self.state == "turn_left":
        if (pose-0.005 < self.turn_num and pose +0.005 > self.turn_num):
          self.state = "move_forward"
          self.turn_state += 1
          if self.turn_state == 8:
            self.turn_state = 0
        else:
          self.turn_left()
      self.rate.sleep() # sleep

def main():
  """Main function"""
  movement = SquareMovement(str(sys.argv[1])) # name of the robot
  movement.move()
  rospy.spin()

if __name__ == "__main__":
  # initialize node
  rospy.init_node('publisher_cmd', anonymous=True)
  try:
    main()
  except rospy.ROSInterruptException:
    print("Shutting down publisher")
    pass