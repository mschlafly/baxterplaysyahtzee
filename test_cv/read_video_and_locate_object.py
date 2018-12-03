#!/usr/bin/env python2
# -*- coding: utf-8 -*-

# Read baxter's camera's video, detect chessboard and locate object at the same time.

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from matplotlib import pyplot as plt
from std_msgs.msg import String

import os, sys
import time
PACKAGE_PATH = os.path.join(os.path.dirname(__file__))+"/../"
# sys.path.append(PACKAGE_PATH+"")

# our written massage
from baxterplaysyahtzee.msg import ColorBound 


# ---------------------- Import from our own library -----------------------
from ourlib_cv import ChessboardLocator, Object3DPoseLocator, find_object, myTrackbar

# Set CAMERA_TOPIC: the camera topic to read video from Baxter
USE_HEAD_CAMERA = False
if USE_HEAD_CAMERA:
    tmp_CAMERA_TOPIC = "head_camera"
    STR_CAMERA_TYPE = "Head"
else:
    tmp_CAMERA_TOPIC = "left_hand_camera"
    STR_CAMERA_TYPE = "Left"
CAMERA_TOPIC = "/cameras/"+tmp_CAMERA_TOPIC+"/image"

### Use my laptop camera
# $ rosrun usb_cam usb_cam_node _video_device:=/dev/video0 _pixel_format:=yuyv _camera_name:=tracker_camera
# CAMERA_TOPIC= "/usb_cam/image_raw" # use my laptop camera

# Other settings
IMAGE_FILENAME_FOR_SAVING = 'leftcamera_detection_result'
USE_TEST_OBJECT_POS_IN_IMAGE=False
SQUARE_SIZE=0.0982/5 # This is the real square size of the chessboard 

# Low bound and up bound for color thresholding
global COLOR_LB, COLOR_UB
COLOR_LB=(36, 45, 0)
COLOR_UB=(79, 255, 255)

class BaxterCameraProcessing(object):
    def __init__(self):
        print("Initialing class BaxterCameraProcessing")
        self.bridge = CvBridge()

        # Params
        self.cnt = 0  # count images

        # Set a subscriber
        self.sub = rospy.Subscriber(CAMERA_TOPIC, Image, self.main_loop)

        # Set algorithms instances
        self.chessboard_locator = ChessboardLocator(STR_CAMERA_TYPE, SQUARE_SIZE=SQUARE_SIZE)

    def main_loop(self, msg):

        self.cnt += 1

        # Convert ROS Image message to OpenCV2
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        rospy.loginfo("\nReading the {}th image".format(self.cnt))

        # Image processing
        flag_result, img_for_display = self.Do_Image_Processing(img)
        if flag_result:
            # print("Find object")
            None

        # Show image
        cv2.imshow("img_for_display", img_for_display)
        cv2.waitKey(1)

        # Save image
        IF_WRITE_TO_FILE = False
        SKIP = 20
        if IF_WRITE_TO_FILE and self.cnt % SKIP == 0:
            str_image_idx = "{:05d}".format(self.cnt)
            filename = PACKAGE_PATH+"/images/" + \
                IMAGE_FILENAME_FOR_SAVING+'_'+str_image_idx+'.jpg'
            res = cv2.imwrite(filename, img)
            print(res)
            print("    Save image to: %s" % filename)

    def Do_Image_Processing(self,img):

        # Detect chessboard
        flag, R, p, img_for_display = self.chessboard_locator.locate_chessboard(
            img,SAVE=False, SHOW=False, PRINT=False)

        if flag is False:
            return False, img_for_display
        
        # Detect object in the image
        if USE_TEST_OBJECT_POS_IN_IMAGE:
            xi = 400
            yi = 300
            radius = 20
        else:
            global COLOR_LB, COLOR_UB
            xi, yi, radius, mask=find_object(img, COLOR_LB, COLOR_UB)
            img_for_display=np.hstack([img_for_display,mask])
            if xi is None:
                return False, img_for_display

        # Locate the object 3D pose wrt camera frame and chessboard frame
        op = Object3DPoseLocator(
            STR_CAMERA_TYPE,
            R_cam_table=R,
            p_cam_table=p,
        )
        res_P_camera, res_P_board = op.locate_object(xi=xi, yi=yi, PRINT=True)

        # ---- Show ----
        # Add circle to img_for_display
        img_for_display = cv2.circle(img_for_display,
                                     center=(xi, yi),
                                     radius=radius,
                                     color=[0, 0, 255],
                                     thickness=2, lineType=8, shift=0)

        # Add text to img_for_display
        FONT = cv2.FONT_HERSHEY_SIMPLEX
        sss = ["object pos wrt chessboard", "x=", "y="]
        ppp = [xi, yi]
        for i in range(-1, 2):
            if i != -1:
                s = "{:.2f}".format(res_P_board[i, 0])
            else:
                s = ""
            TEST_ROWS = yi-80+i*30
            TEST_COLS = xi-50
            COLOR = 255
            img_for_display = cv2.putText(
                img_for_display, sss[i+1]+s, (TEST_COLS, TEST_ROWS), FONT,
                0.8, (0, 0, COLOR), 2, cv2.LINE_AA)

        res=True
        return res, img_for_display



def callback_ColorBound(msg):
    global COLOR_LB, COLOR_UB
    COLOR_LB=(msg.low_bound0,msg.low_bound1,msg.low_bound2)
    COLOR_UB=(msg.high_bound0, msg.high_bound1, msg.high_bound2)

if __name__ == '__main__':
    rospy.init_node('read_video_and_locate_object')
    sub=rospy.Subscriber("ColorBound",ColorBound,callback_ColorBound)
    BaxterCameraProcessing()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()