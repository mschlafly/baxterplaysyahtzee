#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
Readme
"""

"""
Import
"""
import re
import sys
import cv2
import csv
import enum
import copy
import numpy
import random
import operator
import webcolors
import argparse
from optparse import OptionParser
import inspect
from sets import Set
from os import system
from requests import get
from contextlib import closing
from collections import Counter
from bs4 import BeautifulSoup as soup
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from baxterplaysyahtzee.msg import GameState, KeepDice

# ros
import rospy
import roslib
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

"""
yahtzee imports
"""
from geometry_msgs.msg import (Point, Pose, PoseStamped, Quaternion)

# cv
#from CalibChessboardPose.srv import Pose
#from GetObjectInBaxter.srv import import Pose

# 
#from GetObjectInImage.srv import XYRadiusAngle
#from GetAllObjectsInImage.srv import flag
from baxterplaysyahtzee.srv import *
from baxterplaysyahtzee.msg import *

# mp 
from std_srvs.srv import Trigger
from baxterplaysyahtzee.srv import *

# ge

# d

"""
Define
"""
NODE_NAME = "yahtzee"
DEBUG = True
GAMESTATE_TOPIC = "/statetopic"
REROLL_TOPIC = "/reroll"
SOME_SRV= "/some/srv"
RATE = 10
QUEUE_SIZE = 10

"""
Class
"""
class Main():
    def __init__(self):
        self.debug_info = DEBUG;
        self.debug(self.fname(inspect.currentframe()))
        rospy.init_node(NODE_NAME)
        self.debug("started node"+NODE_NAME)
        self.pub = rospy.Publisher(GAMESTATE_TOPIC, GameState, queue_size=QUEUE_SIZE)
        self.rate = rospy.Rate(RATE)

    def initbot(self):
        self.debug(self.fname(inspect.currentframe()))

    """
    cv
    """

    def calibrate(self):
        self.debug(self.fname(inspect.currentframe()))
        SERVICE_NAME="/mycvCalibChessboardPose"
        print "calling service: " + SERVICE_NAME
        val = call_service(SERVICE_NAME, CalibChessboardPose)
        self.debug(val)

    def visible_objects(self):
        self.debug(self.fname(inspect.currentframe()))
        SERVICE_NAME="/mycvGetAllObjectsInImage"
        self.debug("calling service: " + SERVICE_NAME)
        val = self.call_service(SERVICE_NAME, GetAllObjectsInImage)
        self.debug(val)

    def object_in_robot(self):
        self.debug(self.fname(inspect.currentframe()))
        SERVICE_NAME="/mycvGetObjectInBaxter"
        self.debug("calling service: " + SERVICE_NAME)
        resp=self.call_service(SERVICE_NAME, GetObjectInBaxter)
        if resp.flag:
            objInfo=resp.objInfo
            print "\n\n -----------Detect the object!------------- \n"
            self.debug(type(objInfo))
            self.debug(objInfo)
            return objInfo
        else:
            print "Not finding anything"

    def object_in_image(self):
        self.debug(self.fname(inspect.currentframe()))
        SERVICE_NAME="/mycvGetObjectInImage"
        self.debug("calling service: " + SERVICE_NAME)
        val = self.call_service(SERVICE_NAME, GetObjectInImage)
        self.debug(val)

    """
    mp
    """
    def move_to_cup_offset(self, pose):
        self.debug(self.fname(inspect.currentframe()))
        cup_pose = Pose(
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

        move_to_cup_offset = rospy.ServiceProxy('iktest_controller/move_to_cup_offset', OffsetMove)
        val = move_to_cup_offset()
        self.debug(val)

    def pick_up_dice_above(self, poses):
        self.debug(self.fname(inspect.currentframe()))
        pick_up_dice_above = rospy.ServiceProxy('iktest_controller/pick_up_dice_above', OffsetMove)

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

        val = pick_up_dice_above(dice_pose)
        self.debug(val)

    def pick_up_dice(self, pose):
        self.debug(self.fname(inspect.currentframe()))
        pick_up_dice = rospy.ServiceProxy('iktest_controller/pick_up_dice', OffsetMove)
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
        val = pick_up_dice(dice_pose)
        self.debug(val)

    def move_to_initpose(self):
        move_to_initpose = rospy.ServiceProxy('iktest_controller/move_to_initpose', Trigger)
        val = move_to_initpose()
        self.debug(val)

    def move_to_homepose(self):
        self.debug(self.fname(inspect.currentframe()))
        move_to_homepose= rospy.ServiceProxy('iktest_controller/move_to_homepose', Trigger)
        val = move_to_homepose()
        self.debug(val)

    def pour_dice(self, pose):
        self.debug(self.fname(inspect.currentframe()))
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
        pour_dice = rospy.ServiceProxy('iktest_controller/pour_dice', OffsetMove)
        val = pour_dice(dice_pose)
        self.debug(val)

    """
    flow
    """
    def home_position(self):
        self.debug(self.fname(inspect.currentframe()))
        pass

    def pickup_cup(self):
        self.debug(self.fname(inspect.currentframe()))
        # send mp location
        pass

    def shake_cup(self):
        self.debug(self.fname(inspect.currentframe()))
        pass

    def pour_dice(self):
        self.debug(self.fname(inspect.currentframe()))
        # pour_dice service

    def place_cup(self):
        self.debug(self.fname(inspect.currentframe()))
        # place cup outside boundary
        #

    def read_values(self):
        self.debug(self.fname(inspect.currentframe()))
        # read dice values
        pass

    def decide_move(self):
        self.debug(self.fname(inspect.currentframe()))
    
    def pickup_dice(self):
        self.debug(self.fname(inspect.currentframe()))
        # send mp location
        pass

    def startup_display(self):
        self.debug(self.fname(inspect.currentframe()))
        update_display()

    def update_display(self):
        self.debug(self.fname(inspect.currentframe()))
        pass

    def fetch_param(self,name,default):
        self.debug(self.fname(inspect.currentframe()))
        if rospy.has_param(name):
            return rospy.get_param(name)
        else:
            rospy.logwarn("parameter %s not defined. Defaulting to %.3f" % (name, default))
        return default

    def publisher(self):
        print("publisher")
        self.debug(self.fname(inspect.currentframe()))

        game_state = GameState()
        game_state.state= "Dice Read"
        game_state.turn = 1# turn number (total=13)
        game_state.roll = 2# roll numer (either 1,2, or 3)
        game_state.dice1 = 4# number of dots on dice1
        game_state.dice2 = 2# number of dots on dice2
        game_state.dice3 = 5# number of dots on dice3
        game_state.dice4 = 3# number of dots on dice4
        game_state.dice5 = 2# number of dots on dice5
        game_state. dice1color = "b" # color of dice1
        game_state.dice2color = "bl"# color of dice2
        game_state.dice3color = "g" # color of dice3
        game_state.dice4color = "y" # color of dice4
        game_state.dice5color = "r" # color of dice5

        while not rospy.is_shutdown():
            self.pub.publish(game_state)
            self.rate.sleep()
    """
    utils
    """

    def debugtool(self):
        inspect.getmembers(Main, predicate=inspect.ismethod)

    def call_service(self, 
                     service_name, 
                     service_type, 
                     args=None):
        rospy.wait_for_service(service_name)
        try:
            func = rospy.ServiceProxy(service_name, 
                                      service_type)
            val = func(*args) if args else func() 
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e
        return val

    def debug(self, msg):
        if(self.debug_info):
            rospy.loginfo(msg)

    def fname(self, frame):
        return inspect.getframeinfo(frame).function

    def reroll_callback(self, data, args):
        print("GOT KEEPERS")
        k = data #KeepDice
        k.dice1
        k.dice2
        k.dice3
        k.dice4
        k.dice5
        print(k)

        # send with value 1 (reroll) in self.list_ofobjects to motion planning to be picked up and put into cup
        # reroll_objects = compare (self.list_of_objects and k)
        #self.pick_up_dice_above(self_objects) #ok

    def listener(self):
        print("listening for reroll....")
        rospy.Subscriber(REROLL_TOPIC, KeepDice, self.reroll_callback, (self))

    def loop(self):
        self.listener()
        try:
            self.publisher()
        except rospy.ROSInterruptException:
            pass
        rospy.spin()

    """
    colors
    """
    def closest_colour(self, requested_colour):
        min_colours = {}
        for key, name in webcolors.css3_hex_to_names.items():
            r_c, g_c, b_c = webcolors.hex_to_rgb(key)
            rd = (r_c - requested_colour[0]) ** 2
            gd = (g_c - requested_colour[1]) ** 2
            bd = (b_c - requested_colour[2]) ** 2
            min_colours[(rd + gd + bd)] = name
        return min_colours[min(min_colours.keys())]

    def get_colour_name(self, requested_colour):
        try:
            closest_name = actual_name = webcolors.rgb_to_name(requested_colour)
        except ValueError:
            closest_name = self.closest_colour(requested_colour)
            actual_name = None
        return actual_name, closest_name


    """
    Test
    """
    def test_cv(self):
        # cv test
        object = self.object_in_image()
        #self.move_to_cup_offset(object.pose) # do not test
        object = self.object_in_robot()
        #self.move_to_cup_offset(object.pose) # do not test
        list_of_objects = self.visible_objects()
        self.pick_up_dice() # ok

        #actual, closest = self.closest_colour((19, 31, 55))
        #print actual, closest
    def test_mp(self):
        self.pick_up_dice_above() #ok
        self.pick_up_dice() # ok
        self.pour_dice() # ok
        self.move_to_homepose() # ok
        #self.move_to_initpose() # do not test
        #self.move_to_cup_offset() # do not test
    """
    example flow
    """
    def example_flow(self):
        # ie get visible dice
        self.list_of_objects = self.visible_objects()

        # send dice values to display code
        self.publish() # should take list_of_dice values, see publish for hardcoded example

       #check self.reroll_callback for response

"""
Init
"""
def main():
    m = Main()
    m.loop()

    #m.test_cv()
    #m.test_mp()

if __name__ == '__main':
    try: 
        main()

    except rospy.ROSInterruptException:
            print "ROS Interrut : %s" %e
            pass
main()
