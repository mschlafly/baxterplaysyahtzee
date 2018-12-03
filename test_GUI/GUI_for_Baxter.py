#!/usr/bin/python

import rospy
from baxter_pykdl import baxter_kinematics
import baxter_interface

from GUI_lib import myTkGUI

def list_to_str(vals):
    str_val = ["{:.3f} ".format(val) for val in vals]
    str_val = ", ".join(str_val)
    return str_val

class myTkGUIforBaxter(myTkGUI):
    def __init__(self):
        super(myTkGUIforBaxter, self).__init__()
    
    def fk_func(self):
        self.set_text(self.fk_OutP, "EndPose = " + "1,1,1")
        self.set_text(self.fk_OutR, "EndAngle= " + "1,2,3")

    def ik_func(self):
        self.set_text(self.ik_Out, "ik result")

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