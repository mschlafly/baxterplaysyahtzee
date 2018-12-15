# baxterplaysyahtzee
# ME495 Final Project: Baxter Robot Plays Yahtzee

# 0. Video of our Project
[video](https://www.youtube.com/watch?v=vOceYSICtQc)

# 1. Introduction

## 1.1 Purpose of Project

The purpose of our project was to create a demonstration showing Baxter playing
a game of Yahtzee against a human opponent. Our goal was to incorporate
different elements from our coursework in ME 495 into our final product.

## 1.2 Workflow of Baxter Playing Yahtzee

Our original plan for our demonstration consisted of six main components listed as follows:

0. Dice start in the cup
1. Baxter picks up cup
2. Baxter shakes cup
3. Baxter pours the dice out of the cup
4. Baxter read values displayed on dice
5. Yahtzee game engine accepts input, and outputs a "decision"
6. Baxter picks up a subset of the dice based on game engine output

## 1.3 Actual Implementation
In our actual demonstration, the playing dice are to start in a cup at the edge of Baxter's "playing space" on a table. The cup will always start at a known location
near Baxter's left arm. Baxter will pick up the cup filled with dice, and then
dump them onto the table. While shaking the cup was part of our original proposal, we realized that it was unnecessary and that turning the wrist joint at the end of the left arm would be sufficient enough to "roll" the dice. We created a physical boundary around Baxter's playing
space on the table in order to confine the dice to an area with somewhat
consistent lighting and in reach of Baxter's arm.
We have a computer vision algorithm running on a node that detects the positions of the dice , as well as their values, using the feed from Baxter's head camera. Baxter then hones in on a single dice and uses its arm camera to calculate a finer position for that die within the work space. We were not able to get a full version of our Yahtzee game engine integrated with the rest of our system, but in theory, a running total would be kept of the values of the rolled dice and then sent to the game engine node. The corresponding "best move" for the game would be found by the game engine based upon this score. Certain dice would then be picked up and placed back in the cup at the edge of the workspace. An implementation of the "best move" would have Baxter rolling a subset of the dice to maximize its score. In our demo, we decided to have Baxter pick up the dice and place them back into the cup at the edge of the workspace after each roll to represent completion of its turn. Further adjustments were made to our repository after demo day to increase our system's robustness.

# 2. How to Run This Package

First, run the node of computer vision:
> $ rosrun baxterplaysyahtzee nodeCV.py

Then, put chessboard on the table and under the camera, and call this service to get the pose of table surface:
> $ rosservice call /mycvCalibChessboard

Finally, launch the package using:
> $ roslaunch baxterplaysyahtzee yahtzee_baxter.launch

The launch file launches 4 nodes:
headdisplay.py
gripper_control.py
iktest.py
sequence.py

Among them, **sequence.py** is the main node that executes the steps in our workflow.


##### Computer Vision

Services from Computer Vision Part：
**/mycvGetObjectInImage**: Detect the dice in the middle of the image, return pos in image.

**/mycvGetAllObjectsInImage**: Detect all dices in image, return pos in image

**/mycvCalibChessboardPose**: Calibrate the pose of table's surface. If there is a chessboard in image, detect its pose wrt camera, and then get and save its pose wrt baxter's base.

**/mycvGetObjectInBaxter**: Call /mycvGetObjectInImage, and then transform pixel pos to world pos.

**/mycvGetAllObjectsInBaxter**: Call /mycvGetAllObjectsInImage, and then transform all objects' pixel pos to world pos.

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

## Package

## Nodes
- What did you want it to do?
- What does it do?
- What would it do with more time?
- Libraries used
- Inputs/Outputs
- Publishers
- Subscribers
- Services Provided or Called
- Any interesting issues or bugs encountered/addressed

<describe nodes here>

## Services
iktest.py
rospy.Service('iktest_controller/pick_up_dice_above', OffsetMove, self.svc_pick_up_dice_above)
rospy.Service('iktest_controller/pick_up_dice', OffsetMove, self.svc_pick_up_dice)
rospy.Service('iktest_controller/move_to_initpose', Trigger, self.svc_move_to_initpose)
rospy.Service('iktest_controller/move_to_homepose', Trigger, self.svc_move_to_homepose)
rospy.Service('iktest_controller/pour_dice', Trigger, self.svc_pour_dice)
rospy.Service('iktest_controller/pour_the_cup', CupShake, self.svc_handle_pour_the_cup)

### Service Messages
srv/

#### CalibChessboardPose.srv

```
---
bool flag # return status
geometry_msgs/Pose pose # pose information for chessboard
```

#### CupShake.srv
```
---
bool succes  # return success
```

#### GetAllObjectsInBaxter.srv
```
---
bool flag # return flag
baxterplaysyahtzee/ObjectInfo[] objInfos # array of objects
```

#### GetAllObjectsInImage.srv
```
---
bool flag
baxterplaysyahtzee/ObjectInfo objInfo
~
```

#### GetObjectInBaxter.srv
```
---
bool flag
baxterplaysyahtzee/ObjectInfo objInfo
~
```

#### GetObjectInImage.srv
```
---
bool flag
baxterplaysyahtzee/ObjectInfo objInfo
```

#### OffsetMove.srv
```
geometry_msgs/Pose pose   # Desired offset position for the motion controller to adjust the current EE position by
---
bool success                # Indicate successful run of triggered service
string message              # Informational, e.g. for error messages
```


## Topics
```
GAMESTATE_TOPIC = "/statetopic"
REROLL_TOPIC = "/reroll"
```

## Publishers
```

```

## Subscribers
```
headdisplay.py to /statetopic
```

## Message Types
ColorBound.msg  
```
float32 low_bound0
float32 low_bound1
float32 low_bound2
float32 high_bound0
float32 high_bound1
float32 high_bound2
```
GameState.msg  
```
string state # state of the system in format for head display (e.g."Rolling Dice"). If the state is "Dice Read", headdisplay.py will display values and determine next move.
int32 turn # turn number (total=13)
int32 roll # roll numer (either 1,2, or 3)
int32 dice1 # number of dots on dice1
int32 dice2 # number of dots on dice2
int32 dice3 # number of dots on dice3
int32 dice4 # number of dots on dice4
int32 dice5 # number of dots on dice5
string dice1color # color of dice1
string dice2color # color of dice2
string dice3color # color of dice3
string dice4color # color of dice4
string dice5color # color of dice5

```
KeepDice.msg  
```
int32 dice1 # booleans for whether to keep (0) for reshake (1) dice
int32 dice2 #
int32 dice3 #
int32 dice4 #
int32 dice5 #
```
ObjectInfo.msg  
```
int32 index
float32 xi
float32 yi
float32 radius_x
float32 radius_y
float32 radius_mean
float32 angle

geometry_msgs/Pose pose

int32 value
string color
```

# Algorithms
## Algorithms for Computer Vision

### 1. Camera Calibration

We only used Baxter's left hand camera. It's camera info is already there in the topic, so no need for calibration.

But we did calibrate its head camera using Python. See /camera_calibration.

### 2. Get the Pose of Table Surface

We used a chessboard to do this. Given the chessboard corners' pos in image, and their real pos in chessboard frame, we solve **PnP** to get the transformation from **camera frame** to **chessboard frame**. By left multiplying another matrix, we get the transformation from **Baxter base frame** to **chessboard frame**.

### 3. Locate Object Pos in Baxter Frame
Suppose we get the object pos in image. We know the object is on the **table surface (left side of equations)**, and it's also on a **beam (right side of equatiosn)** shooted from camera's focal point. We solve the equations (X1=X2, Y1=Y2, 0=Z2) to get X1, Y1. These are the object's pos in Chessboard frame. Transform it to the Baxter's frame.

### 4. Detect All Dices in Image

We used Graph Based Image Segmentation ([paper](http://cs.brown.edu/people/pfelzens/segment/), [code](https://github.com/luisgabriel/image-segmentation)) and square detection to find all dices. If the dice's color and the table's color are enough distinguishable, this can work well.

We resize the image to 320x200 and then calls this algorithm. It takes 4s to compute.

### 5. Detect One Dice in Image

If we know there is a dice in the middle of the image, we first define a potential region it might be in, and then use Grabcut to segment it out.

### 6. Dice Value
The number of dots on the dice surface is the dice value. We use opencv Blob Detection to detect number of dots. 

This algorithm usually works bad. The dots in in dice are small and unclear.

### 7. Dice Color
Currently we get the dice rgb/hsv color by the dice region's median value. (Though not implemented) We can then use kNN and a small training set to determine the dice color.

### 8. Problems

The current performance of our computer vision code doesn't perform as robust as expected. 

For detecting dice, sometimes not all dices are detected. In an ideal condition, with sufficient lighting and uniform table color, the algorithm should work well. However, images from Baxter are dark, and there are also shadows.

For locating dice, there are about 3cm error when the Baxter's hand is 20 cm above the table. It comes from two folds:  
    1. The detected square region in image is not the accurate countour of the real dice.  
    2. The sensor data of Baxter's camera pos might not be so accurate.
    

# 6. Other Notes for Setup or Testing Baxter

### ROS
```
# start roscore in a terminal
roscore

# in another terminal
cd baxterws
source devel/setup.sh
catkin_make

#type roscd and baxterplays then hit tab to ensure ros recognizes the workspace

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
