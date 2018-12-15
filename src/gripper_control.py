#!/usr/bin/env python


import rospy
import baxter_interface

from std_srvs.srv import Trigger


class gripperControls():

    def __init__(self):

        # Baxter Interface Components
        self.left_arm = baxter_interface.Limb('left')
        self.left_wrist = self.left_arm.joint_names()[6]
        self.left_gripper = baxter_interface.Gripper('left')

        self.right_arm = baxter_interface.Limb('right')
        self.right_wrist = self.right_arm.joint_names()[6]
        self.right_gripper = baxter_interface.Gripper('right')

        # Service Definitions
        '''
        rospy.Service('gripper_controller/unscrew_lid', Trigger, self.srv_opening_sequence)
        rospy.Service('gripper_controller/screw_lid', Trigger, self.srv_closing_sequence)

        rospy.Service('gripper_controller/close_grip', Trigger, self.srv_close_grip)
        rospy.Service('gripper_controller/open_grip', Trigger, self.srv_open_grip)
        '''

        rospy.Service('gripper_controller_test/close_grip', Trigger, self.srv_close_grip)
        rospy.Service('gripper_controller_test/open_grip', Trigger, self.srv_open_grip)
    def srv_open_grip(self,data):

        self.left_gripper.open()
        self.right_gripper.open()

        return (True, "GRIP - Gripper Open.")


    def srv_close_grip(self,data):

        self.left_gripper.close()
        self.right_gripper.close()
        print('finish')

        return (True, "GRIP - Gripper Closed.")


# ========== #


def main():

    # Node initialization
    rospy.init_node('gripper_controller_test')
    gripperControls()
    # Class initialization

    while not rospy.is_shutdown():
        rospy.spin()

    return



if __name__ == '__main__':
    print('Has provided services for gripper_control')
    try:
        main()
    except rospy.ROSInterruptException:
        pass
