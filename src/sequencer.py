#!/usr/bin/env python


import rospy

from geometry_msgs.msg import (Point, Pose, PoseStamped, Quaternion)

from std_srvs.srv import Trigger
from baxterplaysyahtzee.srv import OffsetMove

from baxterplaysyahtzee.srv import *
from baxterplaysyahtzee.msg import *

# -------------------------------- Calling Services of Computer Vision --------------------------------
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

def call_cv_service_detect_all_dices():
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

def call_cv_service_detect_one_dice():
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

# -------------------------------- Main --------------------------------


def main():
    global objInfos
    rospy.init_node('sequencer')

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

    '''
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
    '''

    # Main process loop
    while (True):

        # Step 1: pour dices out of the cup
        open_grip()
        move_to_homepose()
        rospy.sleep(1)
        pour_dice()

        # Step 2: Start loop of picking up dices
        cnt_round=0
        FAILURE_TIME=0
        while FAILURE_TIME<10:
            cnt_round+=1
            if cnt_round>5:
                break

            print "---------- THE %dth ROUND ------------"%cnt_round

            move_to_homepose()

            dice_pose = call_cv_service_detect_all_dices()
            if dice_pose is None:
                print "No dice"
                FAILURE_TIME+=1
                continue
            else:
                print "Detect the dice ... "

            def set_dice_orientation(dice_pose):
                dice_pose.orientation.x=0.5
                dice_pose.orientation.y=0.0
                dice_pose.orientation.z=0.0
                dice_pose.orientation.w=-0.0
                print "The dice pos detected by camera is:\n",dice_pose
                return dice_pose

            dice_pose=set_dice_orientation(dice_pose) # set quaternion

            # --------------------------------------
            OFFSET_Z=0.06
            dice_pose.position.z = dice_pose.position.z +OFFSET_Z
            pick_up_dice_above(dice_pose)

            # Refine
            dice_pose = call_cv_service_detect_all_dices()
            # dice_pose = call_cv_service_detect_one_dice()
            if dice_pose is None:
                print "During refining the dice pos, no dice is found."
                FAILURE_TIME+=1
                continue
            else:
                print "Refine dice pos successful"



            # -------------- Publish dice info to the topic
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


            # Gripper goes to the pos which is OFFSET_Z above the dice
            dice_pose=set_dice_orientation(dice_pose)
            dice_pose.position.z = dice_pose.position.z +OFFSET_Z
            pick_up_dice_above(dice_pose)
            rospy.sleep(1)

            # Gripper goes to grab the dice
            dice_pose.position.z = dice_pose.position.z -OFFSET_Z
            print "Height of dice: ", dice_pose.position.z
            pick_up_dice(dice_pose) #//need Offset


        rospy.sleep(1)
        rospy.loginfo("Sequence complete.")
        break

    return


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
