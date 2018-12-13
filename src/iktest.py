#!/usr/bin/env python


import rospy
import baxter_interface
import math
import tf
import struct
import copy

from baxter_core_msgs.msg import EndpointState
from std_msgs.msg import (UInt16,)
from geometry_msgs.msg import (Point, Pose, PoseStamped, Quaternion)
from std_msgs.msg import Header
from baxterplaysyahtzee.srv import*

from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest)
from std_srvs.srv import Trigger

class motionControls():

    def __init__(self):

        # Publishers and Subscribers
        rospy.Subscriber('/robot/limb/left/endpoint_state', EndpointState, self.set_left_ee_position)
        rospy.Subscriber('/robot/limb/right/endpoint_state', EndpointState, self.set_right_ee_position)
        rospy.Subscriber('cup_pose_topic', Pose, self.set_cup_position)
        rospy.Subscriber('dice_pose_topic', Pose, self.set_dice_position)
        rospy.Subscriber('tag_pose_topic', Pose, self.read_tag_position)

        # rospy.Service('iktest_controller/move_to_cup_offset', OffsetMove, self.svc_move_to_cup_offset)
        rospy.Service('iktest_controller/pick_up_dice_above', OffsetMove, self.svc_pick_up_dice_above)
        rospy.Service('iktest_controller/pick_up_dice', OffsetMove, self.svc_pick_up_dice)
        rospy.Service('iktest_controller/move_to_initpose', Trigger, self.svc_move_to_initpose)
        rospy.Service('iktest_controller/move_to_homepose', Trigger, self.svc_move_to_homepose)
        rospy.Service('iktest_controller/pour_dice', Trigger, self.svc_pour_dice)
        rospy.Service('iktest_controller/pour_the_cup', CupShake, self.svc_handle_pour_the_cup)

        self.close_grip = rospy.ServiceProxy('gripper_controller_test/close_grip', Trigger)
        self.open_grip = rospy.ServiceProxy('gripper_controller_test/open_grip', Trigger)
        self.pour_the_cup = rospy.ServiceProxy('iktest_controller/pour_the_cup', CupShake)
        # Just for test
        self.limb = 'left'

        self.dice_above_offset =  Pose(
            position = Point(
            x =  0,
            y =  0,
            z = 0.2),
            orientation = Quaternion())

        self.cup_above_offset = Pose(
            position = Point(
            x =  0,
            y =  0,
            z = 0.2),
            orientation = Quaternion())


        self.dice = Pose(
            position = Point(
            x = 0.823170538072,
            y =   0.379869013763,
            z = 0.0210766529964),
            orientation = Quaternion(
            x = 0.999254863914,
            y = -0.0349792264003,
            z = 0.0143418919002,
            w = 0.00777694036586
                )
        )

 

        self.home_pose = Pose(
            position = Point(
            x =  0.791222167058,
            y = 0.105760404052,
            z = 0.122732076447),
            orientation = Quaternion(
            x = 0.0513302551773,
            y = 0.998019774196,
            z = 0.0328060524923,
            w = -0.0156683801899
                )
        )
        self.cup_above = Pose(
            position = Point(
            x = 0.85190640362,
            y =0.347657681023,
            z =-0.0455096862729
            ),
            orientation = Quaternion(
            x = 0.107378490231,
            y = 0.993110808031,
            z = 0.0308105490348,
            w =-0.0353764452071
                )
        )


        self.cup_ready_to_grip = Pose(
        position = Point(
        x = 0.867223386146,
        y = 0.106406676123,
        z = 0.0761445029937
        ),
        orientation = Quaternion(
        x = 0.700567930833,
        y = -0.100213678564,
        z = 0.697478502247,
        w = 0.112630066237
            ))
        # off3 = off2- off1





        # Service Definitions
        #rospy.Service('ik_test/move_to_cup', Trigger, self.svc_move_to_cup)

    def set_cup_position(self,data):
        self.cup_pose = data
        print(self.cup_pose)
        #print(self.cup)

        return

    def read_tag_position(self, data):
        '''
        Cache new pose values that are obtained from the detected object's AR tag.
        '''

        self.tag = data
        self.home_pose = Pose(
            position = Point(
                x = self.tag.position.x + 1,
                y = self.tag.position.y + 1,
                z = self.tag.position.z + 1
            ),
            orientation = Quaternion(
            x = self.tag.orientation.x + 1,
            y = self.tag.orientation.y + 1,
            z = self.tag.orientation.z + 1,
            w = self.tag.orientation.w + 1)
            )

        return
    def set_dice_position(self,data):
        self.dice = data
        print(self.dice)

        return
    def set_left_ee_position(self, data):
        '''
        Cache pose information for Baxter's left end effector. Useful for offset moves based on current position.
        '''

        self.left_ee_point = data.pose.position
        self.left_ee_orientation = data.pose.orientation

        return


    def set_right_ee_position(self, data):
        '''
        Cache pose information for Baxter's right end effector. Useful for offset moves based on current position.
        '''

        self.right_ee_point = data.pose.position
        self.right_ee_orientation = data.pose.orientation

        return

    def svc_handle_pour_the_cup(self,data):

        # Getting limbs and joints
        #print("Fetching limbs and joints...")
        left_arm = baxter_interface.Limb("left")
        #right_arm = baxter_interface.Limb("right")
        left_joint_names = left_arm.joint_names()
        #right_joint_names = right_arm.joint_names()

        # set joint state publishing to 500Hz
        #print("Setting joint state publishing rate...")
        #rate = 500.0  # Hz
        #pub_rate.publish(rate)

        # set displacement angle for pouring the cup
        #print("Retrieving current joint positions...")
        current_joint_angles = left_arm.joint_angles()
        #print(current_joint_angles)

        #print("Moving the last joint (w2) to its neutral position...")

        current_joint_angles['left_w2'] = 0.0
        left_arm.move_to_joint_positions(current_joint_angles,timeout=15)

        #print("--- Left_w2 at 0.0 ---")
        #print(left_arm.joint_angles())

        print("Moving the last joint by pi/2 radians to pour the dice out...")
        pouring_angles = current_joint_angles
        pouring_angles['left_w2'] = 4*math.pi/7


        left_arm.move_to_joint_positions(pouring_angles,timeout=15)

        # left_arm.set_joint_positions(['left_w2']: math.pi/2)
        #print("--- Joint angles for pouring ---")
        print(left_arm.joint_angles())

        #print("Waiting...")
        rospy.sleep(1.0)

        #print("Moving arm back to standoff position...")
        current_joint_angles = pouring_angles
        current_joint_angles['left_w2'] = 0.0
        left_arm.move_to_joint_positions(current_joint_angles,timeout=15)

        #print("--- Original Standoff position ---")
        print(left_arm.joint_angles())

        rospy.sleep(1.0)

        success = True

        return success

    def add_offset(self,locate,offset):

        add_offset = Pose(
        position = Point(
            x = locate.pose.position.x + offset.position.x,
            y = locate.pose.position.y + offset.position.y,
            z = locate.pose.position.z + offset.position.z
        ),
        orientation = Quaternion(
        x = locate.pose.orientation.x + offset.orientation.x,
        y = locate.pose.orientation.y + offset.orientation.y,
        z = locate.pose.orientation.z + offset.orientation.z,
        w = locate.pose.orientation.w + offset.orientation.w)

        )
        return add_offset


    def move_to_obj(self,data):
        '''
        This service function takes the cached pose information for the , based on the original AR marker's pose,
        calls for an IK solution that brings the designated end effector into position, and executes a move to the
        calculated joint positions.
        '''

        # Establish connection to specific limb's IKSolver service
        ns = '/ExternalTools/' + self.limb + '/PositionKinematicsNode/IKService'
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()

        # Update header information based on current time and with reference to base frame
        hdr = Header(stamp = rospy.Time.now(), frame_id = 'base')

        obj_pose = PoseStamped(
            header = hdr,
            pose = Pose(
                position = Point(
                    x = data.position.x,
                    y = data.position.y,
                    z = data.position.z
                ),
                orientation = Quaternion(
                x = data.orientation.x,
                y = data.orientation.y,
                z = data.orientation.z,
                w = data.orientation.w)
            )
        )


        # Set the desired pose in the service request message to pose information pulled from the object pose topic
        ikreq.pose_stamp.append(obj_pose)


        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call fault: %s" % (e,))
            return (False, "MOTION CTRL - Service call to Baxter's IK solver failed.")
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                                   resp.result_type)


        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
                  (seed_str,))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))

            limb = baxter_interface.Limb(self.limb)
            limb.move_to_joint_positions(limb_joints)
            rospy.loginfo("Valid solution Found! Baxter begins to move!")
        else:
            rospy.loginfo("No valid solution Found!")



        return

    def svc_move_to_initpose(self,data):

        #self.move_to_obj(self.init_pose)

        print ('sucess')
        return (True,'Moving to initial pose')

    def svc_move_to_homepose(self,data):

        #self.move_to_obj(self.test_pose)

        return (True,'Moving to home pose')

        '''
    # has to change this to move to cup above
    def svc_move_to_cup_offset(self,data):

        self.cup_pose_offset = Pose(
            position = Point(
                x = data.pose.position.x + self.cup_pose.position.x,
                y = data.pose.position.y + self.cup_pose.position.y,
                z = data.pose.position.z + self.cup_pose.position.z
            ),
            orientation = Quaternion(
            x = data.offset.orientation.x + self.cup_pose.orientation.x,
            y = data.offset.orientation.y + self.cup_pose.orientation.y,
            z = data.offset.orientation.z + self.cup_pose.orientation.z,
            w = data.offset.orientation.w + self.cup_pose.orientation.w)
            )


        self.move_to_obj(self.cup_pose_offset)

        return (True, 'Moving to cup completed')

         here does not use seed method
        if (resp.isValid[0]):
            rospy.loginfo("IK SOLVER - Success! Valid joint solution found.")

            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))

            # Print Info message to alert users of impending motion
            rospy.loginfo("MOTION CTRL - WARNING! Moving Baxter's " + self.limb + " arm to cup pounce position.")
        '''




    def svc_pick_up_dice_above(self,data):
        print "IM HERE"

        self.dice_above = self.add_offset(data,self.dice_above_offset)


        self.move_to_obj(self.dice_above)

        return (True,"Moving above to dice")

    def svc_pick_up_dice(self,data):
        goal = Pose(
            position = Point(
                x = data.pose.position.x,
                y = data.pose.position.y,
                z = data.pose.position.z
            ),
            orientation = Quaternion(
            x = data.pose.orientation.x,
            y = data.pose.orientation.y,
            z = data.pose.orientation.z,
            w = data.pose.orientation.w)
            )


        self.move_to_obj(goal)

        rospy.sleep(1)

        self.close_grip()

        rospy.sleep(1)

        self.move_to_obj(self.dice_above)

        self.move_to_obj(self.cup_above)

        self.open_grip()

        rospy.loginfo("Have picked up the dice!.")

        rospy.sleep(1)

        return (True,'Dice has been picked up')
        # self.raise_cup()


    def svc_pour_dice(self):

        self.move_to_obj(self.cup_ready_to_grip)
        cup_down = Pose()
        cup_down = Pose(
                position = copy.copy(self.cup_ready_to_grip.position),
                orientation = copy.copy(self.cup_ready_to_grip.orientation))

        cup_down.position.z = self.cup_ready_to_grip.position.z - 0.15

        self.move_to_obj(cup_down)
        rospy.sleep(1)

        self.close_grip()
        rospy.sleep(1)

        self.move_to_obj(self.cup_ready_to_grip)

        self.pour_the_cup()

        rospy.sleep(1)
        self.move_to_obj(cup_down)
        rospy.sleep(1)
        self.open_grip()
        rospy.sleep(1)
        self.move_to_obj(self.cup_ready_to_grip)
        rospy.sleep(1)
        return(True,'Pour finished')



def main():

    # Node initialization
    rospy.init_node('iktest_controller')

    # Class initialization
    iktest_control = motionControls()
    #iktest_control.svc_pour_dice()
    iktest_control.move_to_obj(iktest_control.home_pose)
    #iktest_control.move_to_obj(iktest_control.cup_ready_to_grip)
    #iktest_control.svc_pour_dice()
    #iktest_control.raise_cup()

    #iktest_control.move_to_obj(dice_pose)
    #print(iktest_control.dice_above_offset)
    #iktest_control.move_to_cup(iktest_control.cup)

    while not rospy.is_shutdown():
        rospy.spin()

    return


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
