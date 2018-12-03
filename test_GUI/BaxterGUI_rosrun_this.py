#!/usr/bin/python

# Launch my Baxter GUI

import rospy
from baxter_pykdl import baxter_kinematics

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from BaxterGUI_python_this import myTkGUI


import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

import baxter_interface
from baxter_interface import CHECK_VERSION
from _joint_trajectory_client import Trajectory

def list_to_str(vals):
    str_val = ["{:.3f} ".format(val) for val in vals]
    str_val = ", ".join(str_val)
    return str_val

class myTkGUIforBaxter(myTkGUI):
    def __init__(self):
        super(myTkGUIforBaxter, self).__init__()
    
    # Compute end-effector position and Euler-angles based on given joint angles
    # Currently not supported.
    # def button_callback_fk_func(self):
    #     str_joint_angles = self.get_text(self.fk_In)
    #     self.set_text(self.fk_OutP, "EndPose = " + strP)
    #     self.set_text(self.fk_OutR, "EndAngle= " + strR)

    def read_P_and_R_from_IK_input_text(self):
        str_P = self.get_text(self.ik_InP).split('=')[1].split(',')
        str_R = self.get_text(self.ik_InR).split('=')[1].split(',')
        try:
            P=[float(sval) for sval in str_P]
        except:
            P=[]
        try:
            R=[float(sval) for sval in str_R]
        except:
            R=[]
        return P, R
    
    # compute joint angles (ik) based on given Position (3-vec) and Euler-Angles (3-vec)
    def compute_ik(self, P, R):
        lenP=len(P)
        lenR=len(R)
        joint_angles=None
        if lenP==3 and lenR==3: # x, y, z
            Q=list(quaternion_from_euler(R[0],R[1],R[2]))
            joint_angles=kin.inverse_kinematics(P, Q)
            print("Using both position and angle")
        elif lenP==3:
            joint_angles=kin.inverse_kinematics(P)
            print("Using only position")
        else:
            print("Wrong input!")
        return joint_angles

    # Button press will trigger this function. It reads your input and compute IK.
    def button_callback_ik_compute(self):
        P, R = self.read_P_and_R_from_IK_input_text()
        joint_angles = self.compute_ik(P, R)
        if joint_angles is None:
            print "    IK solutino not found ..."
            self.set_text(self.ik_Out, "IK solutino not found ...")
        else:
            print "    IK: joint angles = ", joint_angles
            self.set_text(self.ik_Out, "Joint angles = " + list_to_str(joint_angles))
            
    # Button press will trigger this function. 
    # It reads your input, compute IK, and move robot to the angles returned by IK.
    def button_callback_ik_move_robot(self):
        P, R = self.read_P_and_R_from_IK_input_text()
        joint_angles = self.compute_ik(P, R)
        if joint_angles is None:
            print "    IK solutino not found ..."            
            return
        if 1: # use set_joint_positions            
            output_angles = dict(zip(joint_names, joint_angles))
            print "\nMoving robot to : ", output_angles
            limb.set_joint_positions(output_angles)
        else: # use traj server
            # positions = {
            #     'left':  [-0.11, -0.62, -1.15, 1.32,  0.80, 1.27,  2.39],
            #     'right':  [0.11, -0.62,  1.15, 1.32, -0.80, 1.27, -2.39],
            # }

            # traj = Trajectory(limb)
            # # Command Current Joint Positions first
            # current_angles = [limb_interface.joint_angle(joint) for joint in joint_names]
            # traj.add_point(current_angles, 0.0)

            # p1 = positions[limb]
            # traj.add_point(p1, 7.0)
            # traj.add_point([x * 0.75 for x in p1], 9.0)
            # traj.add_point([x * 1.25 for x in p1], 12.0)
            # traj.start()
            # traj.wait(15.0)
            # print("Exiting - Joint Trajectory Action Test Complete")

            # joint_names=['left_w0','left_w1','left_w2','left_e0': 0,
            # 		'left_e1': 2.356,'left_s0': -0.7854,'left_s1': -0.7854}

            # 	starting_joint_angles = {'left_w0': 0,'left_w1': -0.7854,'left_w2': 0,'left_e0': 0,
            # 		'left_e1': 2.356,'left_s0': -0.7854,'left_s1': -0.7854}
            None

def loop():
    FLASH_TIME = 0.05
    while not rospy.is_shutdown():

        # Flash text of joint angles
        angles = limb.joint_angles()
        angles = angles.values()
        gui.set_text(gui.show_joint_Out, list_to_str(angles))

        # Flash text of end-effector pose
        # format: [x, y, z, rot_i, rot_j, rot_k, rot_w]
        P_and_Q = kin.forward_position_kinematics()
        position = P_and_Q[0:3]
        quaternion = P_and_Q[3:]
        EulerAngles=list(euler_from_quaternion(quaternion))
        gui.set_text(gui.show_end_OutP, list_to_str(position))
        gui.set_text(gui.show_end_OutR, list_to_str(EulerAngles))

        gui.window.update()
        rospy.sleep(FLASH_TIME)


if __name__ == "__main__":
    rospy.init_node('GUI_for_Baxter')
    print '*** GUI_for_Baxter ***\n'

    ARMS = ['left', 'right']
    ARM_NAME = ARMS[0]

    kin = baxter_kinematics(ARM_NAME)
    limb = baxter_interface.Limb(ARM_NAME)
    limb_interface = baxter_interface.limb.Limb(ARM_NAME)
    joint_names=limb_interface.joint_names()

    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")
  

    gui = myTkGUIforBaxter()

    gui.window.after(100, loop)
    try:
        gui.window.mainloop()
    except:
        pass