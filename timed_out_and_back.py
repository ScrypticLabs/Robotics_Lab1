#!/usr/bin/env python

""" timed_out_and_back.py - Version 0.1 2012-03-24
    A basic demo of the using odometry data to move the robot along
    and out-and-back trajectory.
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
from geometry_msgs.msg import Twist
from math import pi

class OutAndBack():
    def __init__(self):
        # Give the node a name
        rospy.init_node('out_and_back', anonymous=False)

        # Set rospy to exectute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)
        
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist)
        
        # How fast will we update the robot's movement?
        self.rate = 50
        
        self.r = rospy.Rate(self.rate)
        
    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
        
    def translate(self, linear_distance, linear_speed=0.2):
    	linear_speed *= abs(linear_distance)/linear_distance
    	linear_duration = linear_distance/linear_speed
    	
    	move_cmd = Twist()
    	move_cmd.linear.x = linear_speed
    	
    	ticks = int(linear_duration * self.rate)
    	for t in range(ticks):
    		self.cmd_vel.publish(move_cmd)
       		self.r.sleep()
        
    	move_cmd = Twist()
    	self.cmd_vel.publish(move_cmd)
    	rospy.sleep(1)
    	
    def rotate(self, angular_distance, angular_speed=0.1):
		angular_distance *= pi/180
		angular_speed *= abs(angular_distance)/angular_distance
		angular_duration = angular_distance/angular_speed

		move_cmd = Twist()
		move_cmd.angular.z = angular_speed

		ticks = int(angular_duration * self.rate)
		for t in range(ticks):
			self.cmd_vel.publish(move_cmd)
			self.r.sleep()

		move_cmd = Twist()
		self.cmd_vel.publish(move_cmd)
		rospy.sleep(1)	

if __name__ == '__main__':
    try:
    	robot = OutAndBack()
    	while True:
			feedback = raw_input("Enter T for translation, R for rotation, or Q for quit: ")
			if feedback == "R":
				distance = float(raw_input("Angle (degrees): "))
				robot.rotate(distance)
			elif feedback == "T":
				distance = float(raw_input("Distance: "))
				robot.translate(distance)
	  		elif feedback == "Q":
				robot.shutdown()
				break
    except Exception as e:
    	print(e)
        rospy.loginfo("Out-and-Back node terminated.")

