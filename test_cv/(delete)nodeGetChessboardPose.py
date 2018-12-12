#!/usr/bin/env python2
# -*- coding: utf-8 -*-

# Read baxter's camera's video, detect chessboard and locate object at the same time.

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose

import cv2
from matplotlib import pyplot as plt
from std_msgs.msg import String

import os, sys
import time
PACKAGE_PATH = os.path.join(os.path.dirname(__file__))+"/../"
# sys.path.append(PACKAGE_PATH+"")


# ---------------------- Import from our own library -----------------------
from ourlib_cv import ChessboardLocator

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
SQUARE_SIZE=0.0982/5 # This is the real square size of the chessboard 
NODE_NAME="nodeGetChessboardPose"

def set_str_error(str_error):
    return "\n\nError from " +NODE_NAME+": "+str_error+"\n"

# def posetonp(msg):
#     #Given a Pose message for the keurig, convert to numpy arrays p and Q.
#     Qk = np.array([[msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z]])
#     pk = np.array([[msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]]).T
#     return (Qk,pk)

def Rp_to_pose(R,p):
    pose=Pose()

    R_vec, _ = cv2.Rodrigues(R)
    q=quaternion_from_euler(R_vec)    
    pose.orientation.w=q[0]
    pose.orientation.x=q[1]
    pose.orientation.y=q[2]
    pose.orientation.z=q[3]

    pose.position.x=p[0]
    pose.position.y=p[1]
    pose.position.z=p[2]
    return pose

class classGetChessboardPose(object):
    def __init__(self):
        print("Initialing  "+NODE_NAME)
        self.bridge = CvBridge()

        # Params
        self.cnt = 0  # count images

        # Set a subscriber
        self.sub = rospy.Subscriber(CAMERA_TOPIC, Image, self.topic_receive_image_callback)
        self.rosImage=None
        self.t_receive_image=None

        # Set algorithms instances
        self.chessboard_locator = ChessboardLocator(STR_CAMERA_TYPE, SQUARE_SIZE=SQUARE_SIZE)

    def topic_receive_image_callback(self, msg):
        self.rosImage=msg
        self.t_receive_image=rospy.get_time()

    def service_GetChessboardPose(self, req):

        self.cnt += 1
        if self.rosImage is None:
            rospy.loginfo(set_str_error("not receiving any image."))
            return
        if rospy.get_time()-self.t_receive_image>0.5:
            rospy.loginfo(set_str_error("The received image has been long ago."))
            return

        rospy.loginfo("Inside the service of" + NODE_NAME)

        # Convert ROS Image message to OpenCV2
        img = self.bridge.imgmsg_to_cv2(self.rosImage, "bgr8")

        # Image processing
        flag, R, p, img_for_display = self.chessboard_locator.locate_chessboard(
            img, SAVE=False, SHOW=False, PRINT=False)

        if flag is False:
            return None

        # Show image
        IF_SHOW_IMAGE=False
        if IF_SHOW_IMAGE:
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

        # return
        return Rp_to_pose(R,p)


if __name__ == '__main__':
    rospy.init_node(NODE_NAME)
    classGetChessboardPose()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()