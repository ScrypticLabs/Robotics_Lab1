# Usage
## How to run the main script
### From command line

To initiate ROS MASTER, type:
```bash
$ roscore
```
To clone and build the lab 2 repository, type:
```bash
$ cd ~/catkin_ws/
$ git clone git@github.com:ScrypticLabs/Robotics_Lab1.git src
$ catkin_make
$ source devel/setup.bash
```

To launch the TurtleBot in Gazebo, type:
```bash
$ roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$PWD/bug2_0.world
```

To run the python script, type:
```bash
$ cd ~/catkin_ws
$ ./bug2.py
```

# Methods
-Brain: initialize and commands the GPS of the robot. 

-Robot: contorls the motion of the robot. 

-Sensor: uses kinetic device to sense obstacles along the way to the m-line. 

# Video
[Watch demo here](https://www.youtube.com/watch?v=wNQavCcd48c&t=3s)

