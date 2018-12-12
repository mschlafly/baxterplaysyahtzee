#!/usr/bin/env python


import rospy

from geometry_msgs.msg import (Point, Pose, PoseStamped, Quaternion)

from std_srvs.srv import Trigger
from baxterplaysyahtzee.srv import OffsetMove

def main():

    rospy.init_node('sequencer')

    # Service initializations (IMPORTANT!!!)
    '''
    update_obj_pose = rospy.ServiceProxy('pose_relay/update_obj_pose', Trigger)
    rospy.wait_for_service('pose_relay/update_obj_pose', 3.0)

    store_bottle_pose = rospy.ServiceProxy('motion_controller/store_bottle_pose', Trigger)
    move_to_AR_tag = rospy.ServiceProxy('motion_controller/move_to_AR_tag', Trigger)
    move_to_offset = rospy.ServiceProxy('motion_controller/move_to_offset', OffsetMove)
    move_to_bottle = rospy.ServiceProxy('motion_controller/move_to_bottle', Trigger)
    rospy.wait_for_service('motion_controller/move_to_bottle', 3.0)

    close_grip = rospy.ServiceProxy('gripper_controller/close_grip', Trigger)
    open_grip = rospy.ServiceProxy('gripper_controller/open_grip', Trigger)
    unscrew_lid = rospy.ServiceProxy('gripper_controller/unscrew_lid', Trigger)
    screw_lid = rospy.ServiceProxy('gripper_controller/screw_lid', Trigger)
    rospy.wait_for_service('gripper_controller/screw_lid', 3.0)
    '''

    move_to_cup = rospy.ServiceProxy('iktest_controller/move_to_cup', OffsetMove)
    rospy.wait_for_service('iktest_controller/move_to_cup', 3.0)

    close_grip = rospy.ServiceProxy('gripper_controller_test/close_grip', Trigger)
    open_grip = rospy.ServiceProxy('gripper_controller_test/open_grip', Trigger)

    # Stored offset pose information
    test_1 = Pose(
        position = Point(
        x = 0.779983393525,
        y = 0.252447322072,
        z = 0.4120810140999
        ),
        orientation = Quaternion(
                x = 0.76854798235,
                y = 0.495120693063,
                z = 0.6105294616773,
                w = -0.3192102080429
        )
    )
    test_2 = Pose(
        position = Point(
        x = 0.738630511657,
        y = -0.556555331452,
        z = -0.18647989183),
        orientation = Quaternion(
            x = 0.116148506347,
            y = 0.987081133254,
            z = -0.0710489184446,
            w = 0.0844536087706
            )
    )
    test_3 = Pose(
        position = Point(
        x = 0.742458121141,
        y = -0.54910143742,
        z = -0.0879302385051),
        orientation = Quaternion(
            x = 0.125168443887,
            y = 0.988860385041,
            z = -0.0470217320896,
            w = 0.0653984423634
            )
    )
    # catch big cube
    test_4 = Pose(
        position = Point(
        x =  0.759076806601,
        y = -0.53319955873,
        z = -0.202761397763),
        orientation = Quaternion(
        x = 0.178200601229,
        y = 0.981517350503,
        z = -0.068598144119,
        w = 0.0127487649192
            )
    )

    test_5 = Pose(
        position = Point(
            x = 0.816320665722,
            y = -0.163102198437,
            z = -0.19871643757),
        orientation = Quaternion(
    x = 0.00913239760504,
    y = 0.999826999457,
    z = -0.011413878824,
    w = 0.0115019059931
            )
    )



    '''
        baxCtrl = _init_baxter.BaxterCtrls()

        baxCtrl.enable_baxter()
        baxCtrl.camera_setup_head_LH()
        baxCtrl.calibrate_grippers()

        # Move Baxter's arms to home
        baxCtrl.move_arm_to_home('left')
        baxCtrl.move_arm_to_home('right')

        # Display info message communicating initialization state
        rospy.loginfo("Baxter initialization complete.")
    '''
    # Main process loop
    while (True):

        # Opening the gripper to place the cup on the table
        move_to_cup(test_1);
        rospy.sleep(1)
        move_to_cup(test_4);
        rospy.sleep(1)

        close_grip()

        rospy.sleep(1)
        move_to_cup(test_2);
        rospy.sleep(1)
        move_to_cup(test_5);
        open_grip()
        rospy.sleep(1)

        '''
        rospy.sleep(1)
        open_grip()
        rospy.sleep(1)
        close_grip()
        rospy.sleep(1)
        '''


        rospy.loginfo("Sequence complete.")

        break

    return


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
