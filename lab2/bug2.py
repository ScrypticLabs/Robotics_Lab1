#!/usr/bin/env python

# -*- coding: utf-8 -*-
# @Author: abhi
# @Date:   2018-10-05 12:56:59
# @Last Modified by:   abhi
# @Last Modified time: 2018-10-07 05:55:45

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from math import *
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Point


class Sensor():
    def __init__(self):
        self.regions_ = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0,
        }
        self.sub = rospy.Subscriber('/scan', LaserScan, self.clbk_laser)

    def correct(self, array):
        test = []
        for a in array:
            if isnan(a):
                test.append(10) 
            else:
                test.append(a)
        return test

    def clbk_laser(self, msg):
        MSGS = self.correct(msg.ranges)
        self.regions_ = {
            'right':  min(min(MSGS[0:128]), 10),
            'fright': min(min(MSGS[128:256]), 10),
            'front':  min(min(MSGS[256:384]), 10),
            'fleft':  min(min(MSGS[384:512]), 10),
            'left':   min(min(MSGS[512:640]), 10),
        }
        # print("%f | %f | %f | %f | %f" % (self.regions_['left'], self.regions_['fleft'], self.regions_['front'], self.regions_['fright'], self.regions_['right']))

    def is_equal(self, a, b):
        return abs(a-b) < 0.2

    def is_head_on_collision(self):
        d = 0.9
        # print("%f | %f | %f | %f | %f" % (self.regions_['left'], self.regions_['fleft'], self.regions_['front'], self.regions_['fright'], self.regions_['right']))
        return (0.15 < self.regions_['front'] and self.regions_['front']  < d) or (0.15 < self.regions_['fleft'] and self.regions_['fleft']  < d) or (0.15 < self.regions_['fright'] and self.regions_['fright']  < d)

    def get_values(self):
        return "%f | %f | %f | %f | %f" % (self.regions_['left'], self.regions_['fleft'], self.regions_['front'], self.regions_['fright'], self.regions_['right'])

class Brain():
    def __init__(self, goal_position):
        
        self.start_position = None
        self.goal_position = goal_position
        self.sx, self.sy = None, None
        self.gx, self.gy = self.goal_position
        self.m_direction = None
        self.m_direction_vector = None

        # y = m(x) gives y values on the m line for all x-values
        self.m = None
        self.m_infinite = False
        # we need the last position and the current position to determine direction of motion
        # let's assume that we were moving forward (+y direction) at initialization, then ...
        D = 1
        self.x, self.y = None, None
        self.ox, self.oy = None, None
        
        self.direction_vector = None

        # save last collision history
        self.collision_point = None
        self.next_direction = 0.0

        self.MARGIN_OF_ERROR = 0.3
        self.right_sensor_value = 0

        self.M_Y_MARGIN_OF_ERROR = 0.25

    def is_equal(self, a, b):
        return abs(a-b) < self.MARGIN_OF_ERROR

    def is_m_equal(self, a, b):
        print(abs(a-b))
        print(self.M_Y_MARGIN_OF_ERROR)
        return abs(a-b) < self.M_Y_MARGIN_OF_ERROR

    def are_vectors_equal(self, a, b):
        for i in range(len(a)):
            if not self.is_equal(a[i], b[i]):
                return False
        return True

    def set_starting_position(self, position):
        self.start_position = position
        self.sx, self.sy = self.start_position

        if self.sx != self.gx:
            self.m_direction = self.get_slope(self.start_position, self.goal_position)
            self.m = lambda x: self.m_direction*(x-self.sx)+self.sy
        else:
            self.m = lambda x: 0
            self.m_infinite = True
        # y = m(x) gives y values on the m line for all x-values
        
        self.m_direction_vector = self.normalize(self.get_direction(self.start_position, self.goal_position))

        # we need the last position and the current position to determine direction of motion
        # let's assume that we were moving forward (+y direction) at initialization, then ...
        # D = 1
        self.x, self.y = self.sx, self.sy
        # self.ox, self.oy = self.x-D, self.y

        self.update_direction()

    def set_old_position(self, position):
        # we need the last position and the current position to determine direction of motion
        # let's assume that we were moving forward (+y direction) at initialization, then ...
        self.ox, self.oy = position

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
        if self.m_infinite:
            return self.is_equal(self.x, self.gx)
        # print("My Y: "+str(self.y)+"  Y: "+str(self.m(self.x)))
        return self.is_m_equal(self.m(self.x), self.y)

    # every time we are on the m-line, dot current direction vector with m-line direction vector to find displacement angle
    def get_rotation_to_m_line(self):
        x,y = self.direction_vector
        theta = acos(sum([self.direction_vector[i]*self.m_direction_vector[i] for i in range(len(self.direction_vector))]))
        # is theta anticlockwise or clockwise
        new_vector = (cos(theta*x)-sin(theta*y), sin(theta*x)+cos(theta*y))
        return -theta if self.are_vectors_equal(self.m_direction_vector, new_vector) else theta

    def update_position(self, position):
        # if self.are_vectors_equal(position, (self.x,self.y)): return
        self.ox, self.oy = self.x, self.y
        self.x, self.y = position
        self.update_direction()

    def update_direction(self):
        self.direction_vector = self.normalize(self.get_direction((self.ox, self.oy), (self.x, self.y)))

    def set_collision_point(self, position):
        x, y = position
        self.collision_point = position

    def set_next_direction(self, angle):
        self.next_direction = angle

    def get_next_direction(self):
        return self.next_direction

    def remove_collision_point(self):
        self.collision_point = None

    def closer_to_goal_than_collision(self, position):
        cx, cy = self.collision_point
        x, y = position
        distance_from_collision_to_goal = hypot(cx-self.gx,cy-self.gy)
        distance_to_goal = hypot(x-self.gx, y-self.gy)
        # print("original distance: "+str(distance_from_collision_to_goal))
        # print("new distance: "+str(distance_to_goal))
        return distance_to_goal < distance_from_collision_to_goal

    def reached_goal(self):
        return self.are_vectors_equal((self.x, self.y), (self.gx, self.gy))     # we will have to do some tolerance checking

    def has_collided(self):
        return self.collision_point != None

    def clear_memory_of_collision(self):
        self.next_direction = 0
        self.remove_collision_point()

    def remember_right_sensor_value(self, value):
        self.right_sensor_value = value

    def get_memorized_right_sensor_value(self):
        return self.right_sensor_value

class Robot():
    def __init__(self):
        # Give the node a name
        rospy.init_node('robot', anonymous=False)
        # position_ = Point()
        # initial_position_ = Point()
        # initial_position_.x = 0
        # initial_position_.y = 0
        # set robot position
        # model_state = ModelState()
        # model_state.model_name = 'mobile_base'
        # model_state.pose.position.x = initial_position_.x
        # model_state.pose.position.y = initial_position_.y

        # srv_client_set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        # resp = srv_client_set_model_state(model_state)

        # Set rospy to exectute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)
        
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist)

        # Subscriber for current position
        rospy.Subscriber('odom', Odometry, self.update_current_position)

        # How fast will we update the robot's movement?
        self.rate = 50
        self.r = rospy.Rate(self.rate)

        # m-line logic
        self.position = None
        self.brain = Brain(goal_position=(100,0))    # randomly chosen positions
        self.trace = False

        # Sensor Information
        self.sensor = Sensor()

    def run(self):
        counter = 0
        while (not rospy.is_shutdown()):
            if self.get_current_position() != None:
                self.brain.set_old_position(self.get_current_position())
                self.translate(0.1)
                self.brain.set_starting_position(self.get_current_position())
                break

        while (not self.brain.reached_goal()):
            # print(self.sensor.get_values())
            x,y = self.get_current_position()

            if self.trace:
                # print("True: "+str(self.brain.get_memorized_right_sensor_value())+"  Current: "+str(self.sensor.regions_['right']))
                self.wobble(true_sensor_value=self.brain.get_memorized_right_sensor_value(), current_sensor_value=self.sensor.regions_["right"], linear_distance=0.2)
                self.brain.update_position(self.get_current_position())
                
                 if self.brain.on_m_line():
                     if self.brain.closer_to_goal_than_collision((x,y)):
                         self.brain.remove_collision_point()
                         self.trace = False
                         print("go back to m-line")

            if not self.brain.has_collided():
                # print(counter)
                if self.sensor.is_head_on_collision():
                    self.brain.set_collision_point((x,y))
                    self.trace = True
                    counter += 1
                    # print("rotate")
                    while(self.sensor.regions_['front'] < 10):
                        self.rotate(0.2)
                    self.brain.remember_right_sensor_value(self.sensor.regions_['right'])
                    # self.translate_indef()
                    self.translate(0.2)
                    self.brain.update_position(self.get_current_position())
                else:
                    angular_distance = self.brain.get_rotation_to_m_line()
                    print(angular_distance*180/pi)
                    if abs(angular_distance) > 0:
                        # print("Current Direction: "+str(self.brain.direction_vector))
                        # print("M Direction: "+str(self.brain.m_direction_vector))
                        # print("rotating to m-line")
                        self.rotate(angular_distance)
                    self.translate(0.2)
                    self.brain.update_position(self.get_current_position())

                    # self.brain.set_next_direction(-pi/2)
            # elif self.sensor.is_side_free(self.brain.desired_open_side()):
                # self.stop()
                # self.brain.update_position(self.get_current_position())
                # self.rotate(self.brain.get_next_direction())
                # self.brain.clear_memory_of_collision()
            # print(self.trace)
            # self.process_sensor_data()
            # if self.brain.on_m_line():
            #     # print("on m line")
            #     # self.trace = False   # you might not have to trace anymore
            #     if self.trace and (not self.brain.has_collided() or self.brain.closer_to_goal_than_collision((x,y))):
            #         self.brain.remove_collision_point()
            #         angular_distance = self.brain.get_rotation_to_m_line()
            #         # print(angular_distance*180/pi)
            #         if not self.brain.is_equal(angular_distance, 0):
            #             self.rotate(angular_distance)
            #         self.translate(0.1)
            #         self.brain.update_position(self.get_current_position())
            #         self.trace = False
                # write code to detect collision
                # if collision, trace = true, follow the boundary
                # else:                   # you reached the m-line but you aren't any closer to the goal (therefore u are back at the m-line you first encountered at collision)
                    # self.trace = True        # keep tracing boundary
            # if self.trace:
            #     pass


                # if self.front_obstacle_percentage <= self.FRONT_OBSTACLE_THRESHOLD:
                #     if max(self.side_left, self.side_right) > self.SIDE_OBSTACLE_THRESHOLD:
                #         if self.side_left > self.side_right:
                #             self.rotate(self.side_left*pi/2)
                #         else:
                #             self.rotate(-self.side_right*pi/2)
                # # follow boundary
                # # implement trace boundary
                # # do we rotate left or right?
                # if self.front_left_obstacle > self.front_right_obstacle:
                #     print("Left")
                #     # rotate left
                #     self.rotate(self.front_obstacle_percentage*pi/2)
                # else:
                #     print("Right")
                #     # rotate right
                #     self.rotate(-self.front_obstacle_percentage*pi/2)
                # self.translate(self.TRANSLATE_INCREMENT)
            # self.brain.update_position(self.get_current_position())


    def get_current_position(self):
        return self.position

    def update_current_position(self, msg):
        self.position = (msg.pose.pose.position.x,msg.pose.pose.position.y)
        
    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def translate_indef(self, linear_speed=0.2):
        move_cmd = Twist()
        move_cmd.linear.x = linear_speed
        self.cmd_vel.publish(move_cmd)

    def stop(self):
        self.translate(0)

    def wobble(self, true_sensor_value, current_sensor_value, linear_distance, linear_speed = 0.3, angular_speed = 0.5):
        linear_speed *= abs(linear_distance)/linear_distance
        linear_duration = linear_distance/linear_speed
        
        move_cmd = Twist()
        move_cmd.linear.x = linear_speed

        if current_sensor_value > true_sensor_value:
            move_cmd.angular.z = -1*angular_speed
        elif current_sensor_value < true_sensor_value:
            move_cmd.angular.z = angular_speed
        else:
            move_cmd.angular.z = 0
        
        ticks = int(linear_duration * self.rate)
        for t in range(ticks):
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()
        
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
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
   
    def rotate(self, angular_distance, angular_speed=0.2):
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
    except Exception as e:
        print(e)
        rospy.loginfo("Robot node terminated.")
