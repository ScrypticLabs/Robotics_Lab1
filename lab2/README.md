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
$ cd ~/.ros
$ roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$PWD/bug2_0.world
```

To run the python script, type:
```bash
$ cd ~/catkin_ws
$ ./bug2.py
```

# Classes
-Brain: initialize and commands the GPS of the robot. 

-Robot: contorls the motion of the robot. 

-Sensor: uses kinetic device to sense obstacles along the way to the m-line. 

# Video
[Watch bug2_0.world demo here](https://youtu.be/j3KI6px4v3k)

[Watch bug2_1.world demo here](https://www.youtube.com/watch?v=fIg5r1BtLwE)
