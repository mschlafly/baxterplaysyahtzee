# baxterplaysyahtzee
# ME495 Final Project: Baxter Robot Plays Yahtzee

## Setup

### ROS
```
roscore
```
### Baxter
```
rosrun baxter_interface joint_trajectory_action_server.py
rosrun baxter_interface gripper_action_server.py
```
#### Simulator
```
roslaunch baxter_gazebo baxter_world.launch
rosrun baxter_examples xdisplay_image -f jarvis.jpg
rosrun baxter_tools enable_robot.py -e
rosrun baxter_interface joint_trajectory_action_server.py 
```

#### Robot
##### Connect via Ethernet
```
# Specify Baxter's hostname
baxter_hostname="baxter.local"

# Set *Either* your computers ip address or hostname. Please note if using
# your_hostname that this must be resolvable to Baxter.
your_ip="10.42.0.1"
```

##### Camera Calibration
```
python src/detect_checkerboard.py
```
