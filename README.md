# baxterplaysyahtzee
##ME495 Final Project: Baxter Robot Plays Yahtzee

## Setup

### Baxter
```
rosrun baxter_interface joint_trajectory_action_server.py
rosrun baxter_interface gripper_action_server.py
```

#### Simulator
```
roscore
roslaunch baxter_gazebo baxter_world.launch
rosrun baxter_examples xdisplay_image -f jarvis.jpg
rosrun baxter_tools enable_robot.py -e
rosrun baxter_interface joint_trajectory_action_server.py 
```

#### Robot Camera Calibration
```
src/detect_checkerboard.py
```

#### Robot
Connect via Ethernet
```
roscore
```
