#!/usr/bin/env python

# -*- coding: utf-8 -*-
# @Author: abhi
# @Date:   2018-10-05 12:56:59
# @Last Modified by:   abhi
# @Last Modified time: 2018-10-05 17:34:29

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import *


class Brain():
	def __init__(self, start_position, goal_position):
		self.start_position = start_position
		self.goal_position = goal_position
		self.sx, self.sy = self.start_position
		self.gx, self.gy = self.goal_position
		self.m_direction = self.get_slope(self.start_position, self.goal_position)
		self.m_direction_vector = self.normalize(self.get_direction(self.start_position, self.goal_position))

		# y = m(x) gives y values on the m line for all x-values
		self.m = lambda x: self.m_direction*(x-self.sx)+self.sy

		# we need the last position and the current position to determine direction of motion
		# let's assume that we were moving forward (+y direction) at initialization, then ...
		D = 1
		self.x, self.y = self.sx, self.sy
		self.ox, self.oy = self.x, self.y-D
		
		self.direction_vector = None
		self.update_direction()

		# save last collision history
		self.collision_point = None

	def get_slope(self, start_position, end_position):
		ex, ey = end_position
		sx, sy = start_position
		return float(ey-sy)/(ex-sx)

	def get_direction(self, old_position, current_position):
		ox, oy = old_position
		x, y = current_position
		return (x-ox, y-oy)

	def normalize(self, vector):
		magnitude = (sum([i**2 for i in vector]))**0.5
		return tuple([float(i)/magnitude for i in vector])

	def on_m_line(self):
		return self.m(self.x) == self.y

	# every time we are on the m-line, dot current direction vector with m-line direction vector to find displacement angle
	def get_rotation_to_m_line(self):
		x,y = self.direction_vector
		theta = acos(sum([self.direction_vector[i]*self.m_direction_vector[i] for i in range(len(self.direction_vector))]))
		# is theta anticlockwise or clockwise
		return theta if self.m_direction_vector == (cos(theta*x)-sin(theta*y), sin(theta*x)+cos(theta*y)) else -theta

	def update_position(self, position):
		self.ox, self.oy = self.x, self.y
		self.x, self.y = position
		self.update_direction()

	def update_direction(self):
		self.direction_vector = self.normalize(self.get_direction((self.ox, self.oy), (self.x, self.y)))

	def set_collision_point(self, position):
		x, y = position
		self.collision_point = position

	def remove_collision_point(self):
		self.collision_point = None

	def closer_to_goal_than_collision(self, position):
		cx, cy = self.collision_point
		x, y = position
		distance_from_collision_to_goal = hypot(cx-self.gx,cy-self.gy)
		distance_to_goal = hypot(x-self.gx, y-self.gy)
		return distance_to_goal < distance_from_collision_to_goal

	def reached_goal(self):
		return self.x == self.gx and self.y == self.gy 	# we will have to do some tolerance checking


class Robot():
    def __init__(self):
        # Give the node a name
        rospy.init_node('robot', anonymous=False)

        # Set rospy to exectute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)
        
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist)

        # Subscriber for current position
        rospy.Subscriber('odom', Odometry, self.update_current_position)

        # How fast will we update the robot's movement?
        self.rate = 50
        self.r = rospy.Rate(self.rate)

        # m-line logic
        self.position = (0,0)
        self.brain = Brain(start_position=self.position, goal_position=(10,0))	# randomly chosen positions
        self.trace = False

    def run(self):
    	while (not self.brain.reached_goal()):
    	# while (not rospy.is_shutdown()):
			if self.brain.on_m_line():
				print("ON M LINE")
				trace = False 	# you might not have to trace anymore
				if self.brain.collision_point == None or self.brain.closer_to_goal_than_collision():
					self.brain.remove_collision_point()
					angular_distance = self.brain.get_rotation_to_m_line()
					print(angular_distance)
					if angular_distance != 0:
						self.rotate(angular_distance)
					else:
						self.translate(SOME_DISTANCE)
			# write code to detect collision
			# if collision, trace = true, follow the boundary
			else:
				# you reached the m-line but you aren't any closer to the goal (therefore u are back at the m-line you first encountered at collision)
				trace = True # keep tracing boundary
			if trace:
				# follow boundary
				# implement trace boundary
				pass
	# self.brain.update_position(self.get_current_position())

    def get_current_position(self):
    	return self.position

    # ROS needs to give us a real-time position
    def update_current_position(self, msg):
    	self.position = (msg.pose.pose.position.x,msg.pose.pose.position.y)
        
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
		# angular_distance *= pi/180
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

    	robot = Robot()
    	robot.run()


    	"""
    	while True:
			feedback = raw_input("Enter T for translation, R for rotation, or Q for quit: ")
			if feedback == "R":
				distance = float(raw_input("Angle (degrees): "))
				robot.rotate(distance)
			elif feedback == "T":
				distance = float(raw_input("Distance: "))
				robot.translate(distance)
	  		else:
				robot.shutdown()
				break
		"""
    except Exception as e:
    	print(e)
        rospy.loginfo("Robot node terminated.")
