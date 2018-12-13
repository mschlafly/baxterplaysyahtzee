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
SOME_CALLBACK = "callback()"
NODE_NAME = "yahtzee"
DEBUG = True
SOME_TOPIC = "/some/topic"
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
        self.pub = rospy.Publisher(SOME_SRV, Twist, queue_size=QUEUE_SIZE)
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
    def move_to_cup_offset(self):
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

    def pick_up_dice_above(self):
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

    def pick_up_dice(self):
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

    def pour_dice(self):
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
        """
        ge srv
        """
        # update display
        pass
    
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
        self.debug(self.fname(inspect.currentframe()))
        pub = rospy.Publisher('SOME_TOPIC',Twist, queue_size=QUEUE_SIZE)
        rate = rospy.Rate(10)
        time = 0.0

        T = rospy.get_param('~T', 10)
        twist = Twist()

        while not rospy.is_shutdown():
            pub.publish(twist)
            rate.sleep()
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

    def callback(self, data, args):
        return inspect.getframeinfo(frame).function
        client = args
        v = data
        x = v.x
        y = v.y

        # a method
        client.commands_from_coords(x, y)

    def commands_from_coords(self, x, y):
        return inspect.getframeinfo(frame).function
        global servo_x, servo_y

    def listener(self):
        return inspect.getframeinfo(frame).function
        rospy.Subscriber('SOME_TOPIC', Image, self.SOME_CALLBACK, (self))

    def loop():
        return inspect.getframeinfo(frame).function
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
        self.object_in_image()
        object = self.object_in_robot()
        actual, closest = self.closest_colour((19, 31, 55))
        print actual, closest
        self.visible_objects()

    def test_mp(self):
        self.pick_up_dice_above() #ok
        self.pick_up_dice() # ok
        self.pour_dice() # ok
        self.move_to_homepose() # ok
        #self.move_to_initpose() # do not test
        #self.move_to_cup_offset() # do not test
"""
Init
"""
def main():
    m = Main()
    m.test_mp()

if __name__ == '__main':
    try: 
        main()

    except rospy.ROSInterruptException:
            print "ROS Interrut : %s" %e
            pass
main()
