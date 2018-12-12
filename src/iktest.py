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
        rospy.Subscriber('iktest_topic', Pose, self.set_cup_position)

        rospy.Service('iktest_controller/move_to_cup', OffsetMove, self.svc_move_to_cup)
        # Just for test
        self.limb = 'right'
        self.cup = Pose()

        # Service Definitions
        #rospy.Service('ik_test/move_to_cup', Trigger, self.svc_move_to_cup)

    def set_cup_position(self,data):
        self.cup = data
        print(self.cup)
        #print(self.cup)

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

    def svc_move_to_cup(self,data):
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
        '''
        pose_test2 = Pose(
            position = Point(
            x = 0.779983393525,
            y = 0.252447322072,
            z = 0.0120810140999
            ),
            orientation = Quaternion(
                    x = 0.86854798235,
                    y = 0.495120693063,
                    z = 0.0105294616773,
                    w = -0.0192102080429
            )
        )
        '''

        cup_pose = PoseStamped(
            header = hdr,
            pose = Pose(
                position = Point(
                    x = data.offset.position.x,
                    y = data.offset.position.y,
                    z = data.offset.position.z
                ),
                orientation = Quaternion(
                x = data.offset.orientation.x,
                y = data.offset.orientation.y,
                z = data.offset.orientation.z,
                w = data.offset.orientation.w)
            )
        )


        # Set the desired pose in the service request message to pose information pulled from the object pose topic
        ikreq.pose_stamp.append(cup_pose)


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

            # need to be modified

            limb = baxter_interface.Limb('right')
            limb.move_to_joint_positions(limb_joints)
            rospy.loginfo("MOTION CTRL - WARNING! Moving Baxter's " + self.limb + " arm to cup pounce position.")

            ''' here does not use seed method
            if (resp.isValid[0]):
                rospy.loginfo("IK SOLVER - Success! Valid joint solution found.")

                # Format solution into Limb API-compatible dictionary
                limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))

                # Print Info message to alert users of impending motion
                rospy.loginfo("MOTION CTRL - WARNING! Moving Baxter's " + self.limb + " arm to cup pounce position.")
            '''

            return (True, "MOTION CTRL - Move to cup pounce position complete.")
        else:
            rospy.loginfo("IK SOLVER - Failed!")
            return (False, "MOTION CTRL - Motion planner failure.")

        rospy.sleep(3)
        # self.raise_cup()

    def raise_cup(self):
        # Establish connection to specific limb's IKSolver service
        ns = '/ExternalTools/' + self.limb + '/PositionKinematicsNode/IKService'
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()

        # Update header information based on current time and with reference to base frame
        hdr = Header(stamp = rospy.Time.now(), frame_id = 'base')

        pose_test2 = Pose()
        pose_test2.position =  Point(
            x = 0.630002412058,
            y =  0.218421146059,
            z = 0.191631553319
        )
        pose_test2.orientation = Quaternion(x = 0.923779308461,
        y = -0.361591770145,
        z = 0.0254450414594,
        w = -0.123433102883)
        cup_pose = PoseStamped(
            header = hdr,
            pose = Pose(
                position = pose_test2.position,
                orientation = pose_test2.orientation
            )
        )


        # Set the desired pose in the service request message to pose information pulled from the object pose topic
        ikreq.pose_stamp.append(cup_pose)


        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call fault: %s" % (e,))
            return (False, "MOTION CTRL - Service call to Baxter's IK solver failed.")

            '''
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

            limb = baxter_interface.Limb('left')
            limb.move_to_joint_positions(limb_joints)
            rospy.loginfo("MOTION CTRL - WARNING! Moving Baxter's " + self.limb + " arm to cup pounce position.")
        '''

            # here does not use seed method
        if (resp.isValid[0]):
            rospy.loginfo("IK SOLVER - Success! Valid joint solution found.")

            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))

            # Print Info message to alert users of impending motion
            rospy.loginfo("MOTION CTRL - WARNING! Moving Baxter's " + self.limb + " arm to cup pounce position.")

            if (self.limb == 'left'):
                left = baxter_interface.Limb('left')
                left.move_to_joint_positions(limb_joints)
            elif (self.limb == 'right'):
                right = baxter_interface.Limb('right')
                right.move_to_joint_positions(limb_joints)

            return (True, "MOTION CTRL - Move to cup pounce position complete.")
        else:
            rospy.loginfo("IK SOLVER - Failed!")
            return (False, "MOTION CTRL - Motion planner failure.")

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
