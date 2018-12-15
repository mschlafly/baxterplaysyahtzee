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


# 3. Description of Nodes

## 3.1 Main node: ./src/sequence.py

This is our main node that sets up the workflow, e.g. first pour dice, then detect dice, then pick up dice, etc.

It calles the services of motions to move the Baxter's arm, and calles the services of visions to detect the dices on table.

## 3.2 Node for IK and Motion: ./src/iktest.py

This node provides the services for IK and Baxter's Motion, which are:

'''
rospy.Service('iktest_controller/pick_up_dice_above', OffsetMove, self.svc_pick_up_dice_above)  
rospy.Service('iktest_controller/pick_up_dice', OffsetMove, self.svc_pick_up_dice)  
rospy.Service('iktest_controller/move_to_initpose', Trigger, self.svc_move_to_initpose)  
rospy.Service('iktest_controller/move_to_homepose', Trigger, self.svc_move_to_homepose)  
rospy.Service('iktest_controller/pour_dice', Trigger, self.svc_pour_dice)  
rospy.Service('iktest_controller/pour_the_cup', CupShake, self.svc_handle_pour_the_cup)  
'''

## 3.3 Node for Computer Vision: ./src/cv/nodeCV.py

This node provides the following 5 services:

**/mycvGetObjectInImage**: Detect the dice in the middle of the image, return pos in image. (By GrabCut.)

**/mycvGetAllObjectsInImage**: Detect all dices in image, return pos in image. (By another [Graph-based segmentation algorithm](http://cs.brown.edu/people/pfelzens/segment/) and detecting squares).

**/mycvCalibChessboardPose**: Calibrate the pose of table's surface. If there is a chessboard in image, detect its pose wrt camera, and then obtain and save its pose wrt baxter's base.

**/mycvGetObjectInBaxter**: Call /mycvGetObjectInImage, and then transform pixel pos to world pos.

**/mycvGetAllObjectsInBaxter**: Call /mycvGetAllObjectsInImage, and then transform all objects' pixel pos to world pos.

## 3.4 Head Display

## 3.5 The Future Main Node

We are writing a new main node that integrates the motion, vision, dice game engine, and head display. This hasn't been completed.

# 4. Topics / Messages Definition

## 4.1 GameState.msg  


This message contains the current state of our game. It's for the topic "/statetopic", which was intended to be published from the main node to the "headdisplay.py", so that the state info can be shown on the Baxter's head display. 

The definition of this message is:

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

## 4.2 KeepDice.msg  

Booleans for whether to keep (0) for reshake (1) dice
 
``` 
int32 dice1  
int32 dice2  
int32 dice3  
int32 dice4  
int32 dice5  
```

## 4.3 ObjectInfo.msg

After "nodeCV.py" detects dices in the image and locate their positions, it stores the dices' info in this data type.

```
int32 index # The ith dice detected in the image

float32 xi # pos in image
float32 yi
float32 radius_x
float32 radius_y
float32 radius_mean
float32 angle

geometry_msgs/Pose pose # pos in world

int32 value # number of dots
string color
```

## 4.1 ColorBound.msg  

This is for the node of "src/cv/useful_nodes_and_funcs/use_track_bar_to_select_color.py". This node reads in Baxter's video, and then creates a trackbar for setting the HSV/RGB threshold for color thresholding the image. (This is not used in our final implementation).

```
float32 low_bound0
float32 low_bound1
float32 low_bound2
float32 high_bound0
float32 high_bound1
float32 high_bound2
```

# 5 Service Definitions

#### 5.1 CalibChessboardPose.srv

```
---
bool flag # return status
geometry_msgs/Pose pose # pose information for chessboard
```

#### 5.2 CupShake.srv
```
---
bool succes  # return success
```

#### 5.3 GetAllObjectsInBaxter.srv
```
---
bool flag # return flag
baxterplaysyahtzee/ObjectInfo[] objInfos # array of objects
```

#### 5.4 GetAllObjectsInImage.srv
```
---
bool flag
baxterplaysyahtzee/ObjectInfo objInfo
~
```

#### 5.5 GetObjectInBaxter.srv
```
---
bool flag
baxterplaysyahtzee/ObjectInfo objInfo
~
```

#### 5.6 GetObjectInImage.srv
```
---
bool flag
baxterplaysyahtzee/ObjectInfo objInfo
```

#### 5.7 OffsetMove.srv
```
geometry_msgs/Pose pose   # Desired offset position for the motion controller to adjust the current EE position by
---
bool success                # Indicate successful run of triggered service
string message              # Informational, e.g. for error messages
```


# 6. Scripts that Hasn't Been Integrated into Main Node

## 6.1 Game Engine: ./src/game.py

This is python script that runs the simulation and game engine for the Yahtzee Game.


# 7 Algorithms

## 7.1 Algorithms for Computer Vision

### 7.1.1 Camera Calibration

We only used Baxter's left hand camera. It's camera info is already there in the topic, so no need for calibration.

But we did calibrate its head camera using Python. See /camera_calibration.

### 7.1.2 Get the Pose of Table Surface

We used a chessboard to do this. Given the chessboard corners' pos in image, and their real pos in chessboard frame, we solve **PnP** to get the transformation from **camera frame** to **chessboard frame**. By left multiplying another matrix, we get the transformation from **Baxter base frame** to **chessboard frame**.

### 7.1.3 Locate Object Pos in Baxter Frame
Suppose we get the object pos in image. We know the object is on the **table surface (left side of equations)**, and it's also on a **beam (right side of equatiosn)** shooted from camera's focal point. We solve the equations (X1=X2, Y1=Y2, 0=Z2) to get X1, Y1. These are the object's pos in Chessboard frame. Transform it to the Baxter's frame.

### 7.1.4 Detect All Dices in Image

We used Graph Based Image Segmentation ([paper](http://cs.brown.edu/people/pfelzens/segment/), [code](https://github.com/luisgabriel/image-segmentation)) and square detection to find all dices. If the dice's color and the table's color are enough distinguishable, this can work well.

We resize the image to 320x200 and then calls this algorithm. It takes 4s to compute.

### 7.1.5 Detect One Dice in Image

If we know there is a dice in the middle of the image, we first define a potential region it might be in, and then use Grabcut to segment it out.

### 7.1.6 Dice Value
The number of dots on the dice surface is the dice value. We use opencv Blob Detection to detect number of dots. 

This algorithm usually works bad. The dots in in dice are small and unclear.

### 7.1.7 Dice Color
Currently we get the dice rgb/hsv color by the dice region's median value. (Though not implemented) We can then use kNN and a small training set to determine the dice color.

### 7.1.8 Problems

The current performance of our computer vision code doesn't perform as robust as expected. 

For detecting dice, sometimes not all dices are detected. In an ideal condition, with sufficient lighting and uniform table color, the algorithm should work well. However, images from Baxter are dark, and there are also shadows.

For locating dice, there are about 3cm error when the Baxter's hand is 20 cm above the table. It comes from two folds:  
    1. The detected square region in image is not the accurate countour of the real dice.  
    2. The sensor data of Baxter's camera pos might not be so accurate.


------------------------------------------- Notes for Setting or Testing Baxter ---------------------------------
# 7. Notes for Setting or Testing Baxter 

#### Before Running ROS Node
```
# start roscore in a terminal
roscore

# in another terminal
cd baxterws
source devel/setup.sh
catkin_make

#type roscd and baxterplays then hit tab to ensure ros recognizes the workspace

```

#### Useful Servers for Testing Baxter's Motion
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

#### Baxter's Simulator
```
roslaunch baxter_gazebo baxter_world.launch
rosrun baxter_tools enable_robot.py -e
rosrun baxter_examples xdisplay_image.py -f jarvis.jpg
rosrun baxter_interface joint_trajectory_action_server.py
```

Reset World
```
Ctrl+r
```

#### Connection to Baxter via Ethernet

```
cd ~/baxterws/
gedit baxter.sh
```

Setting baxter.sh:
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

#### UI

```
src/rqt_mypkg/resource/MyPlugin.ui # UI description
/src/rqt_mypkg/src/rqt_mypkg # plugin source

rqt --standalone rqt_mypkg
```
