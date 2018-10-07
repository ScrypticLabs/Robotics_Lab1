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

To launch the simulated TurtleBot, type:
```bash
$ catkin_make
$ roslaunch rbx1_bringup fake_turtlebot.launch 
```

To bring up RViz, type:
```bash
$ catkin_make
$ rosrun rviz rviz -d `rospack find rbx1_nav`/sim.rv
```

To run the python script, type:
```bash
$ cd ~/catkin_ws
$ python src/Robotics_Lab1/timed_out_and_back.py
```

# Methods
-Main: constructs a robot object and processes user command until there is an error or the user chooses to quit.

-Init: instantiates a robot with parameters. 

-Shutdown: logs a message and stops the robot.

-Translate: incrementally moves the robot the distance requested by the user.

-Rotate: incrementally rotates the robot the degrees requested by the user.

# Video
[Watch demo here](https://www.youtube.com/watch?v=wNQavCcd48c&t=3s)

