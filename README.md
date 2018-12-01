# baxterplaysyahtzee
# ME495 Final Project: Baxter Robot Plays Yahtzee

## Setup

### ROS
*Make sure to source your environment with source devel/setup.bash *
```
roscore
```
### Baxter


#### Installation

Follow 5.1 Instructions for Building Baxter Software on ROS Melodic in "Baxter and Sawyer Introduction and Resources"

#### Servers
```
rosrun baxter_interface joint_trajectory_action_server.py
rosrun baxter_interface gripper_action_server.py
```

demo trajectory
```
rosrun baxterplaysyahtzee src/joint_trajectory_client.py

# or
python src/joint_trajectory_client.py

# or
rosrun baxter_examples src/joint_trajectory_client.py
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
rqt --standalone rqt_mypkg
```

#### Game
```
cd baxterplays/yahtzee/src
python game.py
```
