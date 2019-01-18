
This is a simple GUI for viewing Baxter's current state, and for easier testing IK and FK.   
(If I'm going to do a Baxter related project in Spring quarter, I might add more functions and make this useful.)

Video: see **video_demo_of_GUI.mp4** inside this folder.  
Image:  
![](screenshot_of_GUI.png)

## 0. Reference  
The file "_joint_trajectory_client.py" and "_joint_position_keyboard.py" are copied from Baxter's official tutorial.  

## 1. Test the GUI by python
You can test and view the GUI by:  
$ python BaxterGUI_python_this.py   

## 2. Run the GUI by ROS

Please run this first:  
> $ roscore (If using laptop)  
> $ roslaunch baxter_gazebo baxter_world.launch  

Launch it on Baxter:  
> $ roslaunch baxterplaysyahtzee launch_GUI_and_KeyboardControl.launch   

This will launch 2 nodes:  
1. GUI (BaxterGUI_rosrun_this.py), currently set to display left arm's info.
2. keyboard input (_joint_position_keyboard.py). This is copied from **baxter_examples/** package!!!  
The keyboard details are on:  
http://sdk.rethinkrobotics.com/wiki/Joint_Position_Keyboard_Example

On GUI, you can see the real-time:  
* left arm joint angles
* left arm's end-effector positions

There are 3 buttons:
1. Compute FK (not implemented)
2. Compute IK  
Sometimes work, sometimes don't.  
The end-effector pose you input had better be close to Baxter's current pose.
3. move robot to here
After pressing, the program reads in the desired end-effector pose, then computes IK, then move robot  to there.

The close button is at the bottom.

