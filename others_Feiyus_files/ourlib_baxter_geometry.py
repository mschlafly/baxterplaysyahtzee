

'''
# Kinematics
# http://sdk.rethinkrobotics.com/wiki/Baxter_PyKDL
This python package "PyKDL" is amazing.
It contains a wrapped up interface, i.e. one line, to call
    *forward kinematics,
    *inverse kinematics,
    *compute Jacobian (which means we can control velocity),
    etc.
You can launch gazebo, and then run its example code.
Simple and good

install:
$ sudo apt-get install ros-melodic-python-orocos-kdl  ros-melodic-orocos-kinematics-dynamics ros-melodic-kdl-conversions ros-melodic-orocos-kdl

'''

def get_end_effector_current_pose():
    # ...
    return

def compute_end_effector_pose(joint_angles):
    # call forward kinematics
    # ...
    return 

def compute_inverse_kinematics(
        p_e, # position of end-effector (vec3)
        R_e=None, # rotation matrix of end-effector (mat3)
        Q_e=None, # quaternion of end-effector (vec4)
    ):
    if Q_e is None:
        # copy somethign from http://kieranwynn.github.io/pyquaternion/#explicitly-by-rotation-parameters
        Q_e = RotationMatrix_to_Quaternion(R_e)
    # ...
    return 

def get_T_base_to_camera():
    return
