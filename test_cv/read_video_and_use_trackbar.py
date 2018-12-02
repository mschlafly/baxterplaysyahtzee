#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 20 22:36:50 2018

@author: feiyu
"""
import cv2
import numpy as np
import time
import rospy
from ourlib_cv import myTrackbar
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from baxterplaysyahtzee.msg import ColorBound 

# Set CAMERA_TOPIC: the camera topic to read video from Baxter
USE_HEAD_CAMERA = False

if USE_HEAD_CAMERA:
    tmp_CAMERA_TOPIC = "head_camera"
    STR_CAMERA_TYPE = "Head"
else:
    tmp_CAMERA_TOPIC = "left_hand_camera"
    STR_CAMERA_TYPE = "Left"
CAMERA_TOPIC = "/cameras/"+tmp_CAMERA_TOPIC+"/image"

# Set vars
global g_img, if_new_data
g_img=None
if_new_data=False


DEFAULT_LB=(36, 45, 0)
DEFAULT_UB=(79, 255, 255)


def sub_to_video(msg):
    img = bridge.imgmsg_to_cv2(msg, "bgr8")
    global g_img, if_new_data
    g_img=img
    if_new_data=True
    print("Read in image")

def pub_color_bound():
    msg.low_bound0=LB[0]
    msg.low_bound1=LB[1]
    msg.low_bound2=LB[2]
    msg.high_bound0=UB[0]
    msg.high_bound1=UB[1]
    msg.high_bound2=UB[2]
    pub.publish(msg)

# Start
if __name__=="__main__":

    rospy.init_node('create_track_bar')
    window_name='track_bar'
    trackbar = myTrackbar(window_name)

    bridge = CvBridge()
    sub = rospy.Subscriber(CAMERA_TOPIC, Image, sub_to_video)
    
    msg=ColorBound()
    pub_topic_name="ColorBound"
    pub = rospy.Publisher(pub_topic_name, ColorBound, queue_size=10)

    while(1):
        if if_new_data==True:
            if_new_data=False
            LB = DEFAULT_LB
            UB = DEFAULT_UB
            hsv = cv2.cvtColor(g_img, cv2.COLOR_BGR2HSV)

            LB, UB=trackbar.get_trackbar_values_LB_UB()
            mask=cv2.inRange(hsv,LB,UB)
            mask = cv2.cvtColor(mask,cv2.COLOR_GRAY2RGB)

            # show
            cv2.imshow(window_name,np.hstack([g_img,mask]))
            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            
            # pub
            pub_color_bound()
        else:
            time.sleep(0.01)    
    cv2.destroyAllWindows()