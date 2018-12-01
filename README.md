# baxterplaysyahtzee
# ME495 Final Project: Baxter Robot Plays Yahtzee

## Setup

### ROS
```
roscore
```
### Baxter


#### Installation

##### Follow 5.1 Instructions for Building Baxter Software on ROS Melodic in "Baxter and Sawyer Introduction and Resources"

##### Make sure to source your environment with source devel/setup.bash

#### Servers
```
rosrun baxter_interface joint_trajectory_action_server.py
rosrun baxter_interface gripper_action_server.py
```

demo trajectory
```
rosrun baxterplaysyahtzee joint_trajectory_client.py -l left

# or
cd src/
python src/joint_trajectory_client.py -l left

# or
rosrun baxter_examples joint_trajectory_client.py -l left
```
#### Simulator
```
roslaunch baxter_gazebo baxter_world.launch
rosrun baxter_tools enable_robot.py -e
rosrun baxter_examples xdisplay_image.py -f jarvis.jpg
rosrun baxter_interface joint_trajectory_action_server.py 
```
##### Reset World
```
Ctrl+r
```

#### Robot
##### Connect via Ethernet

```
cd ~/baxterws/
gedit baxter.sh
```
baxter.sh

Robot 
```
# Specify Baxter's hostname
baxter_hostname="baxter.local"

# Set *Either* your computers ip address or hostname. Please note if using
# your_hostname that this must be resolvable to Baxter.
your_ip="10.42.0.1"
```

Simulator
```
# Specify Baxter's hostname
#baxter_hostname="baxter.local"
baxter_hostname="localhost"

# Set *Either* your computers ip address or hostname. Please note if using
# your_hostname that this must be resolvable to Baxter.
#your_ip="10.42.0.1"
your_hostname="my_computer.local"
```

```
cd ~/baxterws/
source devel/setup.sh
source baxter.sh sim
```

##### Trajectory
http://sdk.rethinkrobotics.com/wiki/Baxter_PyKDL

"A class is provided in this package, baxter_kinematics, which reads from the parameter server of the robot you are connected to pulling the URDF. It then parses that URDF into the expected Orocos kinematic description, a kdl tree. It then, based on the limb specified creates a kinematic chain (base -> gripper frames) on which we will conduct our analysis."

##### Computer Vision
```
roscore
rosrun usb_cam usb_cam_node _video_device:=/dev/video0 _pixel_format:=yuyv _camera_name:=tracker_camera
rosrun baxterplaysyahtzee cv.py
# or
python cv.py
```

##### Camera Calibration
```
cd test_cv
./detect_chessboard.py
./locate_object_3d_pose.py

cd camera_calibration
./calibrate_camerap.py
./undistort_all_images.py
```
#### UI 

```
src/rqt_mypkg/resource/MyPlugin.ui # UI description
/src/rqt_mypkg/src/rqt_mypkg # plugin source

rqt --standalone rqt_mypkg
```

#### Game
```
cd baxterplays/yahtzee/src
python game.py
```
