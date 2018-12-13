#!/usr/bin/env python


import rospy

from geometry_msgs.msg import (Point, Pose, PoseStamped, Quaternion)

from std_srvs.srv import Trigger
from baxterplaysyahtzee.srv import OffsetMove

def main():

    rospy.init_node('sequencer')

    move_to_cup = rospy.ServiceProxy('iktest_controller/move_to_cup', OffsetMove)
    pick_up_dice_above = rospy.ServiceProxy('iktest_controller/pick_up_dice_above', OffsetMove)
    pick_up_dice = rospy.ServiceProxy('iktest_controller/pick_up_dice',Trigger)
    pour_dice = rospy.ServiceProxy('iktest_controller/pour_cup', OffsetMove)

    rospy.wait_for_service('iktest_controller/move_to_cup', 3.0)
    rospy.wait_for_service('iktest_controller/pick_up_dice', 3.0)

    close_grip = rospy.ServiceProxy('gripper_controller_test/close_grip', Trigger)
    open_grip = rospy.ServiceProxy('gripper_controller_test/open_grip', Trigger)



    # Stored offset pose information
    pick_up_dice_offset = Pose(
        position = Point(
        x = 0,
        y = 0,
        z = 0.10
        ),
        orientation = Quaternion()
    )
    pick_up_cup_offset = Pose(
        position = Point(
        x = 0,
        y = 0,
        z = 0.10
        ),
        orientation = Quaternion()
    )
    cup_pose = Pose(
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


        pick_up_dice_above(pick_up_dice_offset)

        rospy.sleep(1)
        pick_up_dice()

        rospy.loginfo("Sequence complete.")

        break

    return


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
