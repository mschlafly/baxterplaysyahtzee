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
class main():
    def __init__(self):
        rospy.init_node('yahtzee')

        self.vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        def example_service(self, x, y, theta):
            rospy.wait_for_service('/turtle1/teleport_absolute')
            try:
                teleport_absolute = rospy.ServiceProxy('SOME_TOPIC',TeleportAbsolute)
                teleport_absolute(x, y, theta)
            except rospy.ServiceException, e:
                print "Service call failed: %s" %e

        def calibrate(self):
            """
            CalibChessboardPose
            geometry_msgs/Pose pose
            """
            pass

        def get_visible_objects(self):
             """
             GetAllObjectsInImage.srv
             """

        def locate_object(self):
             """
             GetObjectInBaxter.srv
             """

        def object_in_image(self):
            """
            GetObjectInImage.srv
            baxterplaysyahtzee/XYRadiusAngle xyra
            """
            pass

        def offset_move(self):
            pass

        def home_position(self):
            pass

        def pickup_cup(self):
            pass

        def shake_cup(self):
            pass

        def pour_dice(self):
            # get wrist angle
            # set wrist angle

            # get wrist angle
            # set wrist angle

            # place cup outside boundary

            pass

        def read_values(self):
            # read dice values
            pass

        def decide_move(self):
            """
            ge srv
            """
            pass
        
        def pickup_dice(self):
            pass

        def update_display(self):
            """
            """
            pass

        def fetch_param(self,name,default):
            if rospy.has_param(name):
                return rospy.get_param(name)
            else:
                rospy.logwarn("parameter %s not defined. Defaulting to %.3f" % (name, default))
            return default

def run(self):
        pub = rospy.Publisher('turtle1/cmd_vel',Twist, queue_size=10)
        rate = rospy.Rate(10)
        time = 0.0

        T = rospy.get_param('~T', 10)
        twist = Twist()

        while not rospy.is_shutdown():

            # calculate linear velocity
            pub.publish(twist)

            rate.sleep()

"""
Init
"""
if __name__ == '__main':
    try: 
        main = main()
        main.main()
    except rospy.ROSInterruptException:
            pass

main()
