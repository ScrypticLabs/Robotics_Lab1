Usage
How to run the main script

To initiate ROS MASTER, type:
$ roscore

To clone and build the lab 2 repository, type:

$ cd ~/catkin_ws/
$ git clone git@github.com:ScrypticLabs/Robotics_Lab1.git src -- CHANGE
$ catkin_make
$ source devel/setup.bash

To launch the simulated TurtleBot in Gazebo, type:
$ roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$PWD/bug2_0.world

To run the python script, type:
$ ./bug2.py

Classes
-Brain: initialize and commands the GPS of the robot. 

-Robot: contorls the motion of the robot. 

-Sensor: uses kinetic device to sense obstacles along the way to the m-line. 

Video
