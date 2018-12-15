

# Simulator Installation
http://sdk.rethinkrobotics.com/wiki/Simulator_Installation

# Gazebo
$ roslaunch baxter_gazebo baxter_world.launch

To start and switch the controllers, use the JointCommand topic as documented in the [Baxter SDK](http://sdk.rethinkrobotics.com/wiki/API_Reference#Arm_Joints_Control_2).

show joint angles in rqt:
$ roslaunch baxter_sim_hardware baxter_sdk_position_rqt.launch

# Examples

### Start Wobbler example:
$ rosrun baxter_examples joint_velocity_wobbler.py

### Start keyboard joint position example:
$ rosrun baxter_examples joint_position_keyboard.py

## Inverse Kinematics
http://sdk.rethinkrobotics.com/wiki/IK_Service_Example

<!-- --------------------- -->

# Forward Kinematics
1. read from topic
rostopic echo /robot/limb/<side>/endpoint_state 
(the Python APIs for both robot's SDKs have nice interfaces to these topics as well)

2. get from tf
robot_state_publisher + tf

# Traj generator
http://sdk.rethinkrobotics.com/wiki/Joint_Trajectory_Action_Server

<!--  -->
# PyKDL
http://sdk.rethinkrobotics.com/wiki/Baxter_PyKDL



# camera control

http://sdk.rethinkrobotics.com/wiki/API_Reference#cameras
search： You can access Baxter's two hand cameras and t

intrinsics:
http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html

camera control:
http://sdk.rethinkrobotics.com/wiki/Camera_Control_Tool


camera details:
http://sdk.rethinkrobotics.com/wiki/Cameras


commands:

view image:
$ rosrun image_view image_view image:=/cameras/left_hand_camera/image

  /cameras/head_camera/camera_info
  /cameras/head_camera/image
  /cameras/right_hand_camera/camera_info
  /cameras/right_hand_camera/image
  /cameras/left_hand_camera/camera_info
  /cameras/left_hand_camera/image