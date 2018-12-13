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
# from CalibChessboardPose.srv import Pose
#from GetObjectInBaxter.srv import import Pose

# tbd
#from GetObjectInImage.srv import XYRadiusAngle
#from GetAllObjectsInImage.srv import flag

# mp tbd
#from std_srvs.srv import Trigger
#from baxterplaysyahtzee.srv import OffsetMove

# ge

# d

"""
Define
"""
DEBUG = True
SOME_TOPIC = "/some/topic"
SOME_SRV= "/some/srv"

"""
Class
"""
class Main():
    def __init__(self):
        self.debug_info = DEBUG;
        self.debug(self.fname(inspect.currentframe()))
        rospy.init_node('yahtzee')

        self.vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

    def debug(self, msg):
        if(self.debug_info):
            rospy.loginfo(msg)

    def fname(self, frame):
        return inspect.getframeinfo(frame).function

    def example_service(self, x, y, theta):
        self.debug(self.fname(inspect.currentframe()))
        rospy.wait_for_service('/turtle1/teleport_absolute')
        try:
            teleport_absolute = rospy.ServiceProxy('SOME_TOPIC',TeleportAbsolute)
            teleport_absolute(x, y, theta)
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e

    def calibrate(self):
        self.debug(self.fname(inspect.currentframe()))
        """
        CalibChessboardPose
        geometry_msgs/Pose pose
        """

        pass

    def get_visible_objects(self):
        self.debug(self.fname(inspect.currentframe()))
        """
        GetAllObjectsInImage.srv
        """
        self.debug("======" + inspect.currentframe() + "======")

    def locate_object(self):
        self.debug(self.fname(inspect.currentframe()))
        """
        GetObjectInBaxter.srv
        """

    def object_in_image(self):
        self.debug(self.fname(inspect.currentframe()))
        """
        GetObjectInImage.srv
        baxterplaysyahtzee/XYRadiusAngle xyra
        """
        debug(self, 
             "======" + inspect.currentframe() + "======")
        pass

    def offset_move(self):
        self.debug(self.fname(inspect.currentframe()))
        pass

    def home_position(self):
        self.debug(self.fname(inspect.currentframe()))
        pass

    def pickup_cup(self):
        self.debug(self.fname(inspect.currentframe()))
        pass

    def shake_cup(self):
        self.debug(self.fname(inspect.currentframe()))
        pass

    def pour_dice(self):
        self.debug(self.fname(inspect.currentframe()))
        # get wrist angle
        # set wrist angle

        # get wrist angle
        # set wrist angle

        # place cup outside boundary

        pass

    def read_values(self):
        self.debug(self.fname(inspect.currentframe()))
        # read dice values
        pass

    def decide_move(self):
        self.debug(self.fname(inspect.currentframe()))
        """
        ge srv
        """
        pass
    
    def pickup_dice(self):
        self.debug(self.fname(inspect.currentframe()))
        pass

    def update_display(self):
        self.debug(self.fname(inspect.currentframe()))
        """
        """
        pass

    def fetch_param(self,name,default):
        self.debug(self.fname(inspect.currentframe()))
        if rospy.has_param(name):
            return rospy.get_param(name)
        else:
            rospy.logwarn("parameter %s not defined. Defaulting to %.3f" % (name, default))
        return default

    def run(self):
        self.debug(self.fname(inspect.currentframe()))
        pub = rospy.Publisher('turtle1/cmd_vel',Twist, queue_size=10)
        rate = rospy.Rate(10)
        time = 0.0

        T = rospy.get_param('~T', 10)
        twist = Twist()

        while not rospy.is_shutdown():
            pub.publish(twist)
            rate.sleep()

    def debugtool(self):
        # inspect.getmembers(Main, predicate=inspect.ismethod)
        pass

"""
Init
"""
def main():
    m = Main()
    m.locate_object()

if __name__ == '__main':
    try: 
        main()

    except rospy.ROSInterruptException:
            pass

main()
