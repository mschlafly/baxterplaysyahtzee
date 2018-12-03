#!/usr/bin/python

import rospy
from baxter_pykdl import baxter_kinematics
import baxter_interface
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from BaxterGUI_python_this import myTkGUI

def list_to_str(vals):
    str_val = ["{:.3f} ".format(val) for val in vals]
    str_val = ", ".join(str_val)
    return str_val

class myTkGUIforBaxter(myTkGUI):
    def __init__(self):
        super(myTkGUIforBaxter, self).__init__()
    
    # def button_callback_fk_func(self): # currently not supported
    #     str_joint_angles = self.get_text(self.fk_In)
    #     self.set_text(self.fk_OutP, "EndPose = " + strP)
    #     self.set_text(self.fk_OutR, "EndAngle= " + strR)

    def button_callback_ik_func(self):
        # compute joint angles based on given end-effector position and Euler-angles
        str_P = self.get_text(self.ik_InP).split('=')[1].split(',')
        str_R = self.get_text(self.ik_InR).split('=')[1].split(',')
        try:
            P=[float(sval) for sval in str_P]
            lenP=len(P)
        except:
            lenP=0
        try:
            R=[float(sval) for sval in str_R]
            lenR=len(R)
        except:
            lenR=0

        if lenP==3 and lenR==3: # x, y, z
            Q=list(quaternion_from_euler(R[0],R[1],R[2]))
            joint_angles=kin.inverse_kinematics(P, Q)
            print("Using both position and angle")
        elif lenP==3:
            joint_angles=kin.inverse_kinematics(P)
            print("Using only position")
        else:
            print("Wrong input!")
            return
        if joint_angles is None:
            print "    IK solutino not found ..."
            self.set_text(self.ik_Out, "IK solutino not found ...")
        else:
            print "    IK: joint angles = ", joint_angles
            self.set_text(self.ik_Out, "Joint angles = " + list_to_str(joint_angles))

def loop():
    FLASH_TIME = 0.05
    while not rospy.is_shutdown():

        # Fresh joint angles
        angles = limb.joint_angles()
        angles = angles.values()
        gui.set_text(gui.show_joint_Out, list_to_str(angles))

        # Fresh end-effector pose
        # format: [x, y, z, rot_i, rot_j, rot_k, rot_w]
        P_and_Q = kin.forward_position_kinematics()
        position = P_and_Q[0:3]
        quaternion = P_and_Q[3:]
        gui.set_text(gui.show_end_OutP, list_to_str(position))
        gui.set_text(gui.show_end_OutR, list_to_str(quaternion))

        gui.window.update()
        rospy.sleep(FLASH_TIME)


if __name__ == "__main__":
    rospy.init_node('GUI_for_Baxter')
    print '*** GUI_for_Baxter ***\n'
    print ""

    ARMS = ['left', 'right']
    ARM = ARMS[0]

    kin = baxter_kinematics(ARM)

    limb = baxter_interface.Limb(ARM)

    gui = myTkGUIforBaxter()

    gui.window.after(100, loop)
    try:
        gui.window.mainloop()
    except:
        pass