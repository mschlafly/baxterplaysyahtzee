#!/usr/bin/env python


import rospy

from geometry_msgs.msg import (Point, Pose, PoseStamped, Quaternion)

from std_srvs.srv import Trigger
from baxterplaysyahtzee.srv import OffsetMove

from baxterplaysyahtzee.srv import *
from baxterplaysyahtzee.msg import *

# -------------------------------- Added by Feiyu --------------------------------
global objInfos

# a template for calling service
def call_service(service_name, service_type, args=None):
    rospy.wait_for_service(service_name)
    try:
        func = rospy.ServiceProxy(service_name, service_type)
        return func(args) if args else func() # call this service
    except rospy.ServiceException, e:
        print "Failed to call service:", service_name
        print "The error is: ", e
        sys.exit()

def call_feiyu_service_detect_all():
    global objInfos
    SERVICE_NAME="/mycvGetAllObjectsInBaxter"
    print "calling service: " + SERVICE_NAME
    resp=call_service(SERVICE_NAME, GetAllObjectsInBaxter)
    if resp.flag:
        objInfos=resp.objInfos
        print "Detect %d objects!\n"%(len(objInfos))
        for i in range(len(objInfos)):
            print "\n\n ------- Printing the %dth pose:-------"%i
            print objInfos[i]

        pose=objInfos[-1].pose
        return pose

    else:
        print "Not finding anything"
        # assert(0)
        return None

def call_feiyu_service_detect_one():
    SERVICE_NAME="/mycvGetObjectInBaxter"
    print "calling service: " + SERVICE_NAME
    resp=call_service(SERVICE_NAME, GetObjectInBaxter)
    if resp.flag:
        objInfo=resp.objInfo
        print "Detect objects!\n"
        print objInfo
        pose=objInfo.pose
        return pose

    else:
        print "Not finding anything"
        return None
        # assert(0)

# -------------------------------- Added by Feiyu Ends here --------------------------------


def main():
    global objInfos
    rospy.init_node('sequencer')

    move_to_initpose = rospy.ServiceProxy('iktest_controller/move_to_initpose', Trigger)
    move_to_homepose = rospy.ServiceProxy('iktest_controller/move_to_homepose', Trigger)
    move_to_cup_offset = rospy.ServiceProxy('iktest_controller/move_to_cup_offset', OffsetMove)
    pick_up_dice_above = rospy.ServiceProxy('iktest_controller/pick_up_dice_above', OffsetMove)
    pick_up_dice = rospy.ServiceProxy('iktest_controller/pick_up_dice',OffsetMove)
    pour_dice = rospy.ServiceProxy('iktest_controller/pour_dice', Trigger)

    #rospy.wait_for_service('iktest_controller/move_to_cup_offset', 3.0)
    rospy.wait_for_service('iktest_controller/pick_up_dice', 3.0)

    close_grip = rospy.ServiceProxy('gripper_controller_test/close_grip', Trigger)
    open_grip = rospy.ServiceProxy('gripper_controller_test/open_grip', Trigger)



    # Stored offset pose information
    pick_up_dice_offset = Pose(
        position = Point(
        x = 0,
        y = 0,
        z = 0.30
        ),
        orientation = Quaternion()
    )
    pick_up_cup_offset = Pose(
        position = Point(
        x = 0,
        y = 0,
        z = 0
        ),
        orientation = Quaternion()
    )
    dice_pose = Pose(
        position = Point(
        x = 0.776647640113,
        y =-0.0615496888226,
        z = -0.210376983209),
        orientation = Quaternion(
            x = 0.981404960951,
            y = -0.19031770757,
            z = 0.016510737149,
            w = -0.0187314806041
            )
    )
    test_3 = Pose(
        position = Point(
        x = 0.00424581141,
        y = -0.0054910142,
        z = -0.0008795051),
        orientation = Quaternion(
            x = 0.00125168443887,
            y = 0.00988860385041,
            z = -0.000470217320896,
            w = 0.000653984423634
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


    pub = rospy.Publisher('/statetopic', GameState, queue_size=10)
    pretend=GameState()

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


        # pick_up_dice_above(pick_up_dice_offset)
        open_grip()

        #move_to_initpose()

        move_to_homepose()


        rospy.sleep(1)

        #move_to_cup_offset(pick_up_cup_offset) # //need Offset

        # ------- added by feiyu ---------------
        pour_dice()

        cnt_round=0
        FAILURE_TIME=0
        while FAILURE_TIME<10:
            cnt_round+=1
            if cnt_round>5:
                break
            print "---------- THE %dth ROUND ------------"%cnt_round

            move_to_homepose()

            dice_pose = call_feiyu_service_detect_all()
            if dice_pose is None:
                print "No dice"
                FAILURE_TIME+=1
                continue
            else:
                print "Detect the dice ... "

            #dice_pose = call_feiyu_service_detect_one()
            #dice_pose.position.x=0.834
            #dice_pose.position.y=0.07

            def reset_dice_pose(dice_pose):
                dice_pose.orientation.x=0.5
                dice_pose.orientation.y=0.0
                dice_pose.orientation.z=0.0
                dice_pose.orientation.w=-0.0
                print "printing feiyu's returned dice-Pose",dice_pose
                return dice_pose
            # set quaternion
            dice_pose=reset_dice_pose(dice_pose)

            # --------------------------------------
            OFFSET_Z=0.06
            dice_pose.position.z = dice_pose.position.z +OFFSET_Z
            pick_up_dice_above(dice_pose)

            # refine
            dice_pose = call_feiyu_service_detect_all()
            # dice_pose = call_feiyu_service_detect_one()
            if dice_pose is None:
                print "During the refine, there is no dice"
                FAILURE_TIME+=1
                continue
            else:
                print "Refine pos successful"

            dice_pose=reset_dice_pose(dice_pose)
            dice_pose.position.z = dice_pose.position.z +OFFSET_Z
            pick_up_dice_above(dice_pose)

            rospy.sleep(1)

            dice_pose.position.z-=OFFSET_Z # restore z back
            dice_pose.position.z = -0.189655249947
            print "Height of dice: ", dice_pose.position.z

            # --------------
            pretend.state="Dice Read"
            pretend.turn=1
            pretend.roll=cnt_round
            pretend.dice1=1
            pretend.dice2=2
            pretend.dice3=3
            pretend.dice4=4
            pretend.dice5=5
            pretend.dice1color='b'
            pretend.dice2color='b'
            pretend.dice3color='r'
            pretend.dice4color='bl'
            pretend.dice5color='y'

            nobj=len(objInfos)
            nthobj=0
            if nobj>nthobj:
                pretend.dice1=objInfos[nthobj].value
                pretend.dice1color=objInfos[nthobj].color
            nthobj=1
            if nobj>nthobj:
                pretend.dice1=objInfos[nthobj].value
                pretend.dice1color=objInfos[nthobj].color
            nthobj=2
            if nobj>nthobj:
                pretend.dice1=objInfos[nthobj].value
                pretend.dice1color=objInfos[nthobj].color
            nthobj=3
            if nobj>nthobj:
                pretend.dice1=objInfos[nthobj].value
                pretend.dice1color=objInfos[nthobj].color
            nthobj=4
            if nobj>nthobj:
                pretend.dice1=objInfos[nthobj].value
                pretend.dice1color=objInfos[nthobj].color
            # --------------

            pick_up_dice(dice_pose) #//need Offset


        # pour_dice()
        # pick_up_dice(pick_up_cup_offset)
        # pour_dice(pick_up_cup_offset) #//need Offset


        rospy.sleep(1)
        # pick_up_dice()

        rospy.loginfo("Sequence complete.")

        break

    return


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
