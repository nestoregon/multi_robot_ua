#!/usr/bin/env python
"""
@author: retama
"""


import rospy
from std_msgs.msg import Int8

def main():
  """Main function"""
  rospy.init_node('node',anonymous=True)
  rate=rospy.Rate(10)       
  pub=rospy.Publisher('custom_topic',Int8,queue_size=10)
  number=0
  rate.sleep()
  pub.publish(Int8(number)) 
  
  while not rospy.is_shutdown():
      rate.sleep()
      

if __name__ == "__main__":
  
  try:
    main()
  except rospy.ROSInterruptException:
    print("Shutting down publisher")
    pass