#!/usr/bin/env python


import rospy
import baxter_interface
import math
import tf
import struct

from baxter_core_msgs.msg import EndpointState
from geometry_msgs.msg import (Point, Pose, PoseStamped, Quaternion)
from std_msgs.msg import Header
from baxterplaysyahtzee.srv import OffsetMove

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

        rospy.Service('iktest_controller/move_to_cup_offset', OffsetMove, self.svc_move_to_cup_offset)
        rospy.Service('iktest_controller/pick_up_dice_above', OffsetMove, self.svc_pick_up_dice_above)
        rospy.Service('iktest_controller/pick_up_dice', OffsetMove, self.svc_pick_up_dice)
        rospy.Service('iktest_controller/move_to_initpose', Trigger, self.svc_move_to_initpose)
        rospy.Service('iktest_controller/move_to_homepose', Trigger, self.svc_move_to_homepose)
        rospy.Service('iktest_controller/pour_dice', OffsetMove, self.svc_pour_dice)

        self.close_grip = rospy.ServiceProxy('gripper_controller_test/close_grip', Trigger)
        self.open_grip = rospy.ServiceProxy('gripper_controller_test/open_grip', Trigger)
        # Just for test
        self.limb = 'right'

        self.cup_pose = Pose(
            position = Point(
            x =  0.785972474799,
            y = -0.274381410551,
            z = -0.20952233235),
            orientation = Quaternion(
            x = 0.029607883567,
            y = 0.99939738394,
            z = -0.0179203375188,
            w = -0.00266527806519
                )
        )

        self.dice = Pose(
            position = Point(
            x =  0.785972474799,
            y = -0.274381410551,
            z = -0.20952233235),
            orientation = Quaternion(
            x = 0.029607883567,
            y = 0.99939738394,
            z = -0.0179203375188,
            w = -0.00266527806519
                )
        )

        self.home_pose = Pose(
            position = Point(
            x =  0.785972474799,
            y = -0.274381410551,
            z = -0.20952233235),
            orientation = Quaternion(
            x = 0.029607883567,
            y = 0.99939738394,
            z = -0.0179203375188,
            w = -0.00266527806519
                )
        )


        print(self.dice)


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
        print(use_advanced_options)
        
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



        return

    # has to change this to move to cup above
    def svc_move_to_cup_offset(self,data):


        self.cup_pose_offset = Pose(
            position = Point(
                x = data.offset.position.x + self.cup_pose.position.x,
                y = data.offset.position.y + self.cup_pose.position.y,
                z = data.offset.position.z + self.cup_pose.position.z
            ),
            orientation = Quaternion(
            x = data.offset.orientation.x + self.cup_pose.orientation.x,
            y = data.offset.orientation.y + self.cup_pose.orientation.y,
            z = data.offset.orientation.z + self.cup_pose.orientation.z,
            w = data.offset.orientation.w + self.cup_pose.orientation.w)
            )


        self.move_to_obj(self.cup_pose_offset)

        return (True, 'Moving to cup completed')

        ''' here does not use seed method
        if (resp.isValid[0]):
            rospy.loginfo("IK SOLVER - Success! Valid joint solution found.")

            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))

            # Print Info message to alert users of impending motion
            rospy.loginfo("MOTION CTRL - WARNING! Moving Baxter's " + self.limb + " arm to cup pounce position.")
        '''


    def svc_move_to_initpose(self,data):

        self.move_to_obj(self.home_pose)

        return (True,'Moving to initial pose')

    def svc_move_to_homepose(self,data):

        self.move_to_obj(self.home_pose)

        return (True,'Moving to home pose')

    def svc_pick_up_dice_above(self,data):

        print('suc')
        self.dice_above = Pose(
            position = Point(
            x = data.offset.position.x + self.dice.position.x,
            y = data.offset.position.y + self.dice.position.y,
            z = data.offset.position.z + self.dice.position.z),
            orientation = Quaternion(
            x = data.offset.orientation.x + self.dice.orientation.x,
            y = data.offset.orientation.y + self.dice.orientation.y,
            z = data.offset.orientation.z + self.dice.orientation.z,
            w = data.offset.orientation.w + self.dice.orientation.w
                )
        )


        self.move_to_obj(self.dice_above)

        return (True,"Moving above to dice")

    def svc_pick_up_dice(self,data):

        self.move_to_obj(self.dice)

        rospy.sleep(1)

        self.close_grip()

        rospy.sleep(1)

        self.move_to_obj(self.home_pose)

        self.cup_pose_above = Pose(
            position = Point(
            x = data.offset.position.x + self.cup_pose.position.x,
            y = data.offset.position.y + self.cup_pose.position.y,
            z = data.offset.position.z + self.cup_pose.position.z),
            orientation = Quaternion(
            x = data.offset.orientation.x + self.cup_pose.orientation.x,
            y = data.offset.orientation.y + self.cup_pose.orientation.y,
            z = data.offset.orientation.z + self.cup_pose.orientation.z,
            w = data.offset.orientation.w + self.cup_pose.orientation.w
                )
        )

        self.move_to_obj(self.cup_pose_above)

        self.open_grip()

        rospy.loginfo("Have picked up the dice!.")

        rospy.sleep(1)

        return (True,'Dice has been picked up')
        # self.raise_cup()


    def svc_pour_dice(self,data):


        cup_down = self.cup_pose_offset
        cup_down.position.z = self.cup_pose_offset.position.z - 0.2

        self.move_to_obj(cup_down)

        self.close_grip()

        self.move_to_obj(self.cup_pose_offset)
        ## Pour dice

        self.move_to_obj(cup_down)

        self.open_grip()

        self.move_to_obj(self.cup_pose_offset)

        return(True,'Pour finished')



def main():

    # Node initialization
    rospy.init_node('iktest_controller')

    # Class initialization
    iktest_control = motionControls()
    #iktest_control.raise_cup()

    #iktest_control.move_to_cup(iktest_control.cup)

    while not rospy.is_shutdown():
        rospy.spin()

    return


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
