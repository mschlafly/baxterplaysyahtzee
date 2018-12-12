# baxterplaysyahtzee
# ME495 Final Project: Baxter Robot Plays Yahtzee

## Plan
0. Dice in Cup
1. Pick up Cup
2. Shake Cup
3. Pour Dice
4. Read Values
5. Decide Move
6. Pick up Subset of Dice

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

``` Feiyu's part:

# To test the functions for detecting chessboard or locate object:
    $ python test_cv/detect_chessboard.py
    $ python test_cv/locate_object_3d_pose.py

# To test a proper color threshold using trackbar on an example image:
    $ python test_cv/use_track_bar_to_select_color.py

# To read video from Baxter and do further processing,
please do these steps first (to open the desired camera):
    $ nmcli connection up Rethink
    $ rosrun baxter_tools camera_control.py -l
    $ rosrun baxter_tools camera_control.py -c right_hand_camera
    $ rosrun baxter_tools camera_control.py -o left_hand_camera -r 640x400
    $ rosrun baxter_tools camera_control.py -o head_camera -r 1280x800
    $ cd test_cv

# To simple test if you can read Baxter's video
    $ rosrun baxterplaysyahtzee read_and_save_video_from_baxter.py

# To locate both the chessboard and object in Baxter's video, run these 2 scripts:
    (A demo I shoot is here: VideoDemo/1201_locate_object.mp4)
    $ rosrun baxterplaysyahtzee read_video_and_use_trackbar.py
    $ rosrun baxterplaysyahtzee read_video_and_locate_object.py

```

##### Camera Calibration
```
$ cd camera_calibration
$ ./calibrate_camerap.py
$ ./undistort_all_images.py
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

