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
DEBUG = False 
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

    def get_visible_objects(self):
        self.debug(self.fname(inspect.currentframe()))
        SERVICE_NAME="/mycvGetAllObjectsInImage"
        self.debug("calling service: " + SERVICE_NAME)
        val = call_service(SERVICE_NAME, GetAllObjectsInImage)
        self.debug(val)

    def locate_object(self):
        self.debug(self.fname(inspect.currentframe()))
        SERVICE_NAME="/mycvGetObjectInBaxter"
        self.debug("calling service: " + SERVICE_NAME)
        val = call_service(SERVICE_NAME, GetObjectInBaxter)
        self.debug(val)

    def object_in_image(self):
        self.debug(self.fname(inspect.currentframe()))
        SERVICE_NAME="/mycvGetObjectInImage"
        self.debug("calling service: " + SERVICE_NAME)
        val = self.call_service(SERVICE_NAME, GetObjectInImage)
        self.debug(val)

    """
    mp
    """

    def offset_move(self):
        self.debug(self.fname(inspect.currentframe()))
        #geometry_msgs/Pose offset   # Desired offset position for the motion controller to adjust the current EE position by
        pass

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
Init
"""
def main():
    return inspect.getframeinfo(frame).function
    m = Main()
    m.object_in_image()

if __name__ == '__main':
    try: 
        main()

    except rospy.ROSInterruptException:
            print "ROS Interrut : %s" %e
            pass

main()
