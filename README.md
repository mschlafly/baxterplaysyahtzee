# baxterplaysyahtzee
# ME495 Final Project: Baxter Robot Plays Yahtzee

This package consists of 
-/camera_calibration for getting the camera parameters for distortion
-/test_cv for locating the table  

## Setup

### ROS
```
*Make sure to source your environment with source devel/setup.bash *
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
rosrun baxter_tools enable_robot.py -e
rosrun baxter_examples xdisplay_image.py -f jarvis.jpg
rosrun baxter_interface joint_trajectory_action_server.py 
```
##### Reset World
```
Ctrl+R
```

#### Robot
##### Connect via Ethernet

```
cd ~/baxterws/
gedit baxter.sh
```
baxter.sh
```
# Specify Baxter's hostname
baxter_hostname="baxter.local"

# Set *Either* your computers ip address or hostname. Please note if using
# your_hostname that this must be resolvable to Baxter.
your_ip="10.42.0.1"
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
or
python cv.py
```

##### Camera Calibration
```
python src/detect_checkerboard.py
```
#### UI 
```
rqt --standalone rqt_mypkg
```
