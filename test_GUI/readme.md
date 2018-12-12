# To read video from Baxter and do further processing,
please do these steps first (to open the desired camera):
    $ nmcli connection up Rethink
    $ rosrun baxter_tools camera_control.py -l
    $ rosrun baxter_tools camera_control.py -c right_hand_camera
    $ rosrun baxter_tools camera_control.py -o left_hand_camera -r 640x400
    $ rosrun baxter_tools camera_control.py -o head_camera -r 1280x800
    $ cd test_cv
    
This is the GUI for viewing Baxter's current state, and also for easier testing IK and FK.

Please do this first  
$ roscore (If using laptop)  
> $ roslaunch baxter_gazebo baxter_world.launch  

## Test by python
You can test and view the GUI by:  
$ python BaxterGUI_python_this.py   

## Test on ROS
Launch it on Baxter:  
> $ roslaunch baxterplaysyahtzee launch_GUI_and_KeyboardControl.launch   

This will launch 2 nodes:  
1. GUI (BaxterGUI_rosrun_this.py), currently set to display left arm's info.
2. keyboard input (_joint_position_keyboard.py).

The keyboard details are on:
http://sdk.rethinkrobotics.com/wiki/Joint_Position_Keyboard_Example

On GUI, you can see the real-time:  
* left arm joint angles
* end-effector positions

There are also 2 buttons for computing fk and ik:
1. The forward kinematics has not currently implemented yet.
2. The inverse kinematics is written, but it is no working properly. It returns no solution all the time.

And also 1 button for moving the robot. Press the button "move robot to here" near "Compute IK", its function will first compute IK based on Position and Euler-Angle, then move the robot to that position.

The close button is at the bottom.

