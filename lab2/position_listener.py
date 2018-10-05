#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

pos = None

def odometryCb(msg):
	global pos
	pos = (msg.pose.pose.position.x,msg.pose.pose.position.y)

if __name__ == "__main__":
    rospy.init_node('position_listener', anonymous=True)
    rospy.Subscriber('odom',Odometry,odometryCb)
    while not rospy.is_shutdown():
    	print(pos)