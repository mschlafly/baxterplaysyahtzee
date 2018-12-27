#!/usr/bin/env python

import rospy
import baxter_interface
import math
import tf
import struct
import copy

from baxter_core_msgs.msg import EndpointState,EndEffectorState
from std_msgs.msg import (UInt16,)
from geometry_msgs.msg import (Point, Pose, PoseStamped, Quaternion)
from std_msgs.msg import Header
from baxterplaysyahtzee.srv import*
from sensor_msgs.msg import JointState
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from sensor_msgs.msg import JointState


from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest)
from std_srvs.srv import Trigger



class motionControls():

    def __init__(self):

        # Publishers and Subscribers
        '''
        rospy.Subscriber('/robot/limb/left/endpoint_state', EndpointState, self.set_left_ee_position)
        '''
        rospy.Subscriber('/robot/end_effector/left_gripper/state', EndEffectorState, self.set_gripper_state)

        # rospy.Service('iktest_controller/move_to_cup_offset', OffsetMove, self.svc_move_to_cup_offset)
        rospy.Service('iktest_controller/pick_up_dice_above', OffsetMove, self.svc_pick_up_dice_above)
        rospy.Service('iktest_controller/pick_up_dice', OffsetMove, self.svc_pick_up_dice)
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
            z = 0), # feiyu set this to zero, modfiy later
            orientation = Quaternion())


        self.home_pose = Pose(
            position = Point(
            x =  0.759933783577 - 0.03,
            y = 0.0352817974998,
            z = -0.00811818441266 + 0.015),
            orientation = Quaternion(
            x = 0.0996597531812,
            y = 0.992987939333,
            z = -0.0174706103672,
            w =  0.0611364351959
                )
        )
        self.cup_above = Pose(
            position = Point(
            x = 0.838527299025 - 0.1 + 0.013,
            y = 0.304700678395 + 0.01,
            z = -0.0115082860402
            ),
            orientation = Quaternion(
            x = 0.998839206921,
            y = -0.0457860406655,
            z = 0.0059744733888,
            w = -0.0137179760005
                )
        )

        self.cup_ready_to_grip = Pose(
        position = Point(
        x = 0.8639658962576,
        y = 0.288214295747,
        z =  0.0819743789111
        ),
        orientation = Quaternion(
        x = 0.697345324287,
        y = -0.0729093965645,
        z = 0.711741333725,
        w = 0.0426379227384
            ))

        self.iksuccess = 1

    def set_gripper_state(self,data):
        self.gripping = data.gripping

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

        print("Moving the last joint by pi/2 radians to pour the dice out...")
        pouring_angles = current_joint_angles
        pouring_angles['left_w2'] = 6*math.pi/7


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


    def move_to_obj(self,data):


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

        if (0):
            # The joint seed is where the IK position solver starts its optimization
            ikreq.seed_mode = ikreq.SEED_USER
            seed = JointState()
            seed.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
            seed.position = [0.7, 0.4, -1.7, 1.4, -1.1, -1.6, -0.4]
            ikreq.seed_angles.append(seed)
            # Once the primary IK task is solved, the solver will then try to bias the
            # the joint angles toward the goal joint configuration. The null space is
            # the extra degrees of freedom the joints can move without affecting the
            # primary IK task.
            #ikreq.use_nullspace_goal.append(True)
            # The nullspace goal can either be the full set or subset of joint angles
            goal = JointState()
            goal.name = ['right_j1', 'right_j2', 'right_j3']
            goal.position = [0.1, -0.3, 0.5]

            #ikreq.nullspace_goal.append(goal)

            # The gain used to bias toward the nullspace goal. Must be [0.0, 1.0]
            # If empty, the default gain of 0.4 will be used
            #ikreq.nullspace_gain.append(0.4)

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
            self.iksuccess = 1

        else:
            self.iksuccess = 0
            rospy.loginfo("No valid solution Found!")

            # # New Addition
            # rospy.loginfo("Attemping a new solution...")
            # limb = baxter_interface.Limb(self.limb)
            # current_joint_angles = limb.joint_angles()
            # current_joint_angles['left_s1'] =  current_joint_angles['left_s1'] + 0.01
            # current_joint_angles['left_s0'] =  current_joint_angles['left_s0'] + 0.01

            # limb.move_to_joint_positions(current_joint_angles)
        return


    def svc_move_to_homepose(self,data):

        self.move_to_obj(self.home_pose)

        return (True,'Moving to home pose')

        '''

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

        self.dice_above = Pose()
        self.dice_above.position = copy.copy(data.pose.position)
        self.dice_above.orientation = copy.copy(data.pose.orientation)


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
            x = 1.0,
            y = 0,
            z = 0,
            w = 0)
            )

        adjust_pose = copy.copy(goal)
        self.move_to_obj(goal)

        while self.iksuccess != 1:

            rospy.loginfo('begin to adjust position to find ik solution')

            adjust_pose.position.z += 0.02
            self.move_to_obj(adjust_pose)
            rospy.loginfo('try to move to goal position again')
            self.move_to_obj(goal)

        rospy.sleep(1)

        self.close_grip()

        rospy.sleep(1)

        if self.gripping != 1:
            rospy.loginfo('fail to catch the dice')

            self.open_grip()

            self.move_to_obj(self.dice_above)

            return (False,'Fail to pick the dice up, ready to try again')
        else:
            rospy.sleep(1)

            self.move_to_obj(self.dice_above)

            self.move_to_obj(self.cup_above)

            self.open_grip()

            rospy.loginfo("Have picked up the dice!.")

            rospy.sleep(1)

            return (True,'Dice has been picked up')
        # self.raise_cup()


    def svc_pour_dice(self,data):

        cup_ready_before= Pose(
            position = Point(
            x = 0.71648935751,
            y = 0.319167260011,
            z = 0.0428410189474
            ),
            orientation = Quaternion(
            x = 0.0602146389381,
            y = 0.720798148774,
            z = -0.102458211566,
            w = 0.68288105909
        ))

        self.move_to_obj(cup_ready_before)
        rospy.sleep(1)

        cup_down_before = copy.copy(cup_ready_before)
        cup_down_before.position.z -= 0.15

        self.move_to_obj(cup_down_before)
        cup_down = copy.copy(cup_down_before)
        cup_down.position.x += 0.12
        self.move_to_obj(cup_down)

        rospy.sleep(1)

        self.close_grip()
        rospy.sleep(1)

        print "Lift the grip"
        pour_dice = copy.copy(cup_down)
        pour_dice.position.z += 0.08
        self.move_to_obj(pour_dice)

        self.pour_the_cup()

        pour_dice.position.z -= 0.08

        rospy.sleep(1)
        self.move_to_obj(pour_dice)
        rospy.sleep(1)
        self.open_grip()

        rospy.sleep(1)

        pour_dice.position.x -= 0.12
        self.move_to_obj(pour_dice)
        pour_dice.position.z += 0.15
        self.move_to_obj(pour_dice)

        rospy.sleep(1)
        return(True,'Pour finished')



def main():

    # Node initialization
    rospy.init_node('iktest_controller')

    # Class initialization
    iktest_control = motionControls()
    rospy.loginfo('service is available!')


    while not rospy.is_shutdown():
        rospy.spin()

    return


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
