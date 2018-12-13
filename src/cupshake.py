#!/usr/bin/env python
import rospy
import math
import baxter_interface
from std_msgs.msg import (
    UInt16,
)
from baxterplaysyahtzee.srv import*


def handle_pour_the_cup(data):

    # Getting limbs and joints
    print("Fetching limbs and joints...")
    left_arm = baxter_interface.Limb("left")
    right_arm = baxter_interface.Limb("right")
    left_joint_names = left_arm.joint_names()
    right_joint_names = right_arm.joint_names()

    # set joint state publishing to 500Hz
    print("Setting joint state publishing rate...")
    rate = 500.0  # Hz
    pub_rate.publish(rate)

    # set displacement angle for pouring the cup
    print("Retrieving current joint positions...")
    current_joint_angles = right_arm.joint_angles()
    print(current_joint_angles)

    print("Setting last joint angle to Pi, and moving the joint to this angle...")
    current_joint_angles['right_w2'] = math.pi
    right_arm.move_to_joint_positions(current_joint_angles,timeout=30)

    print("--- Right_w2 at Pi ---")
    print(right_arm.joint_angles())

    print("Moving the last joint by pi/2 radians to pour the dice out...")
    pouring_angles = current_joint_angles
    pouring_angles['right_w2'] = pouring_angles['right_w2'] - math.pi/2
    right_arm.move_to_joint_positions(pouring_angles,timeout=30)

    print("--- Joint angles for pouring ---")
    print(right_arm.joint_angles())

    print("Waiting...")
    rospy.sleep(2.0)

    print("Moving arm back to standoff position...")
    right_arm.move_to_joint_positions(current_joint_angles,timeout=30)

    print("--- Original Standoff position ---")
    print(right_arm.joint_angles())

    rospy.sleep(1.0)

    success = True

    return success
    # return CupShakeResponse(success)
    # return;


    # rotation_for_pouring = (math.pi)/2
    # standoff_angles = right_arm.joint_angle(joint) for joint in right_arm.joint_names()
    #
    # pour_pos = {'right': [standoff_angles[0],standoff_angles[1],standoff_angles[2],standoff_angles[3],\
    # standoff_angles[4],standoff_angles[5] + rotation_for_pouring]}
    #
    # # move to pouring position
    # print("Moving joint")
    # #right_arm.move_to_joint_positions(pour_pos, timeout=30.0)
    # # Wait a few seconds...
    # rospy.sleep()
    # # move back to original "standoff" position
    # # print("Moving back to original position")
    # # print
    # original_pos = {'right': [standoff_angles[0],standoff_angles[1],standoff_angles[2],standoff_angles[3],\
    # standoff_angles[4],standoff_angles[5]]}
    #
    # right_arm.move_to_joint_positions(original_pos, timeout=30.0);
    #
    # # Sleep at the end
    # rate.sleep(1.0)

def pour_the_cup_server():

    # pub_rate = rospy.Publisher('robot/joint_state_publish_rate',UInt16, queue_size=10)

    #rospy.Subscriber('')

    rospy.init_node('pour_the_cup_server',anonymous=True)

    s = rospy.Service('pour_the_cup', CupShake, handle_pour_the_cup)
    print('Ready to pour the cup!')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    pub_rate = rospy.Publisher('robot/joint_state_publish_rate',UInt16, queue_size=10)
    try:
        pour_the_cup_server()
    except rospy.ROSInterruptException:
        pass
