
This is the GUI for viewing Baxter's current state, and also for easier testing IK and FK.

Please do this first
$ roscore (If using laptop)
$ roslaunch baxter_gazebo baxter_world.launch

then, you can test the GUI by:
$ python BaxterGUI_python_this.py 

or run it on Baxter:
$ roslaunch baxterplaysyahtzee launch_GUI_and_KeyboardControl.launch 
This will launch 2 nodes:
    1. GUI (BaxterGUI_rosrun_this.py), currently set to display left arm's info.
    2. keyboard input (_joint_position_keyboard.py).

The keyboard details are on:
http://sdk.rethinkrobotics.com/wiki/Joint_Position_Keyboard_Example

On GUI, you can see the real-time:
    1. left arm joint angles
    2. end-effector positions
There are also 2 buttons for computing fk and ik:
    1. The forward kinematics has not currently implemented yet.
    2. The inverse kinematics is written, but it is no working properly. It returns no solution all the time.
The close button is at the bottom.

