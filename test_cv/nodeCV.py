#!/usr/bin/env python2
# -*- coding: utf-8 -*-

# Provide services for:
# Read baxter's camera's video, detect chessboard and locate object.

TEST_MODE=True

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
CURRENT_PATH=os.path.join( os.path.dirname(__file__) )+"/"
# sys.path.append(PACKAGE_PATH)
# sys.path.append(PACKAGE_PATH+"")

# ------------settings-----------
# TEST_IMAGE_FILENAME=CURRENT_PATH+"/lib_image_seg"+"/imgmid3.png"
TEST_IMAGE_FILENAME=CURRENT_PATH+"/lib_image_seg"+"/image3.png"

# ---------------------- Import from our own library -----------------------
from ourlib_cv import ChessboardLocator, Object3DPoseLocator, find_object, myTrackbar
from lib_image_seg.ourlib_cv2 import refine_image_mask, find_square, extract_rect,\
    find_object_in_middle, find_all_objects, find_all_objects_then_draw
from ourlib_transformations import form_T, get_Rp_from_T


# ---------------------- service provided by this node -----------------------
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose, Point
from baxterplaysyahtzee.msg import ColorBound, ObjectInfo
from baxterplaysyahtzee.srv import GetAllObjectsInImageResponse, GetObjectInBaxterResponse
from baxterplaysyahtzee.srv import *

# dont forget to call: undistortPoints

# ---------------------- setup -----------------------

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
# SQUARE_SIZE=0.1379/7 # This is the real square size of the chessboard 
SQUARE_SIZE=0.137/4 # This is the real square size of the chessboard 

# Low bound and up bound for color thresholding
global COLOR_LB, COLOR_UB
COLOR_LB=(36, 45, 0)
COLOR_UB=(79, 255, 255)

NODE_NAME="nodeCV"
def set_str_error(str_error):
    return "\nError from " +NODE_NAME+": "+str_error+""

class BaxterCameraProcessing(object):
    def __init__(self):
        print("Initialing class BaxterCameraProcessing")
        self.bridge = CvBridge()

        # Params
        self.cnt = 0  # count images
        self.img=None
        self.t_receive_image=None
        if TEST_MODE:
            # filename=CURRENT_PATH+"/lib_image_seg"+"/image3.png"
            self.img=cv2.imread(TEST_IMAGE_FILENAME, cv2.IMREAD_COLOR)
            rospy.loginfo("Test mode: reading image: "+TEST_IMAGE_FILENAME)
            # print self.img
            # cv2.imshow("test image", self.img)
            # cv2.waitKey(1)

        # Set a subscriber for receiving ros image and publish processed image
        self.sub = rospy.Subscriber(CAMERA_TOPIC, Image, self.topic_receive_image_callback)
        self.pub1 = rospy.Publisher("image_with_chessboard",Image, queue_size=10)
        self.pub2 = rospy.Publisher("image_with_object",Image, queue_size=10)

     
        # services 1: calib chessboard (return: Pose)
        s1 = rospy.Service('mycvCalibChessboardPose', CalibChessboardPose, self.srv_CalibChessboardPose)
        self.R_cam_to_chess=None
        self.p_cam_to_chess=None
        self.image_for_display_chessboard=None

        # services 2: get object in image (return: Point)
        s2 = rospy.Service('mycvGetObjectInImage', GetObjectInImage, self.srv_GetObjectInImage)
        self.object_mask=None

        # services 3: get object in image (return: Point)
        s3 = rospy.Service('mycvGetAllObjectsInImage', GetAllObjectsInImage, self.srv_GetAllObjectsInImage)

        # services 4: get object in Baxter
        s4 = rospy.Service('mycvGetObjectInBaxter', GetObjectInBaxter, self.srv_GetObjectInBaxter)
        self.image_for_display_object=None

    def topic_receive_image_callback(self, rosImage):
        self.img = self.bridge.imgmsg_to_cv2(rosImage, "bgr8")
        self.t_receive_image=rospy.get_time()
        self.cnt+=1

    # tested, OK!!!
    def srv_CalibChessboardPose(self, req):
        print("inside the srv_CalibChessboardPose")
        img=self.img.copy()
        if not self.check_if_image_is_valid():
            rospy.loginfo(set_str_error("srv_CalibChessboardPose failed."))
            return

        print(img.shape)
        # Image processing
        self.chessboard_locator = ChessboardLocator(STR_CAMERA_TYPE, SQUARE_SIZE=SQUARE_SIZE)
        flag, R, p, self.image_for_display_chessboard = self.chessboard_locator.locate_chessboard(
            img, SAVE=False, SHOW=False, PRINT=False)
        self.pub_image_chessboard()

        if flag is False:
            print "Fail to calibrate"
            (self.R_cam_to_chess, self.p_cam_to_chess)=(R,p)
            return Pose()
        else:
            print "Successfully calibrate the chessboard, R=",R,"\np=",p
            (self.R_cam_to_chess, self.p_cam_to_chess)=(R,p)
            pose=Rp_to_pose(R,p)
            return pose

    # tested, OK!!!
    def srv_GetAllObjectsInImage(self, req):
        objInfos=self._GetAllObjectsInImage(req)
        return GetAllObjectsInImageResponse(objInfos)
        
    def _GetAllObjectsInImage(self, req):
        print("inside the srv_GetAllObjectsInImage")
        img=self.img.copy()

        if not self.check_if_image_is_valid():
            rospy.loginfo(set_str_error("srv_GetAllObjectsInImage failed."))
            return

        rects, labeled_img=find_all_objects(img)


        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # -------Remove wrong ones ---
        tmp=list()
        for i in range(len(rects)):
            rect=rects[i]
            (center_x, center_y, radius_x, radius_y, angle)  = extract_rect(rect)
            # Criteria for removing wrong objects
            if center_x<250:
                continue
            else:
                tmp.append(rect)
        rects=tmp
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        # Plot all rects, and output to objInfos
        objInfos=list()

        if len(rects)!=0:
            colored_image = find_all_objects_then_draw(rects, labeled_img, IF_PRINT=False)
            self.image_for_display_object=colored_image   
            self.pub_image_object()   

            # output:
            for i in range(len(rects)):
                rect=rects[i]
                (center_x, center_y, radius_x, radius_y, angle)  = extract_rect(rect)
            
                # ---------------------- Output -----------------
                objInfo=ObjectInfo()
                objInfo.flag2d=True
                (objInfo.xi, objInfo.yi, objInfo.radius_x, objInfo.radius_y, objInfo.angle)=\
                    (center_x, center_y, radius_x, radius_y, angle)
                objInfo.radius_mean=(radius_x+radius_y)/2

                objInfos.append(objInfo)
            
            objInfos.sort(key=lambda x: x.radius_mean, reverse=True)
        else:
            None

        return objInfos

    # tested, OK!!!
    def srv_GetObjectInImage(self, req):
        img=self.img.copy()
        if not self.check_if_image_is_valid():
            rospy.loginfo(set_str_error("srv_CalibChessboardPose failed."))
            return

        # Detect object in the image
        mask, rect = find_object_in_middle(img, ratio_RADIUS_TO_CHECK=3, disextend=50)

        # Display image      
        if rect is not None:  
            self.image_for_display_object=cv2.drawContours(img, [rect], 0, [0,0,1], 2)
            self.pub_image_object()

            # save vars
            self.object_mask=mask
            (center_x, center_y, radius_x, radius_y, angle)  = extract_rect(rect)

            # output:
            objInfo=ObjectInfo()

            objInfo.flag2d=True
            (objInfo.xi, objInfo.yi, objInfo.radius_x, objInfo.radius_y, objInfo.angle)=\
                (center_x, center_y, radius_x, radius_y, angle)
            objInfo.radius_mean=(radius_x+radius_y)/2
        else:
            objInfo=ObjectInfo()
            objInfo.flag2d=False
        return objInfo

    def srv_GetObjectInImage_old(self, req): # This is the old version by color thresholding
        img=self.img.copy()
        
        if not self.check_if_image_is_valid():
            rospy.loginfo(set_str_error("srv_CalibChessboardPose failed."))
            return

        # Detect object in the image
        USE_TEST_OBJECT_POS_IN_IMAGE=False
        if USE_TEST_OBJECT_POS_IN_IMAGE:
            xi = 400
            yi = 300
            radius = 20
        else:
            global COLOR_LB, COLOR_UB
            xi, yi, radius, mask=find_object(img, COLOR_LB, COLOR_UB)
            img_for_display=np.hstack([img_for_display,mask])

        self.object_mask=mask
        return Point(xi, yi, radius)

    def srv_GetObjectInBaxter(self, req):
        img=self.img.copy()
        
        if TEST_MODE: # Extract data from the stored chessboard pos
            if self.R_cam_to_chess is None:
                rospy.loginfo(set_str_error("srv_GetObjectInBaxter: Please calib chessboard first."))
                return
            R_cam_to_chess=self.R_cam_to_chess
            p_cam_to_chess=self.p_cam_to_chess
            T_bax_to_cam=np.identity(4)

        poseLocator = Object3DPoseLocator( # initialize
            STR_CAMERA_TYPE,
            R_cam_table=R_cam_to_chess,
            p_cam_table=p_cam_to_chess
        )

        # Detect objects
        poses=list()
        objInfos=self._GetAllObjectsInImage(None)

        n=len(objInfos)
        print "\n\nGetObjectInBaxter: received ", n, " objects."
        # print objInfos

        if n==0:
            return GetObjectInBaxterResponse(poses)

        # change from image frame to world frame
        IF_PLOT=False
        for i in range(n):
            objInfo=objInfos[i]
            (xi, yi, radius_x, radius_y, angle) = \
                (objInfo.xi, objInfo.yi, objInfo.radius_x, objInfo.radius_y, objInfo.angle)
            radius=objInfo.radius_mean

            # Locate the object 3D (x,y,z) wrt camera frame and chessboard frame
            p_cam_to_obj, p_chess_to_obj = \
                poseLocator.locate_object(xi=xi, yi=yi, PRINT=False) # format: (3,1) column vector

            # Locate the objects direction
            # Input: self.object_mask
            T_cam_to_obj=form_T(R=np.identity(3),p=p_cam_to_obj)

            # transform p_in_camera to p_in_world
            T_bax_to_obj=T_bax_to_cam.dot(T_cam_to_obj)            

            # return
            pose=Pose()
            (pose.position.x, pose.position.y, pose.position.z)=\
                (T_bax_to_obj[0,3],T_bax_to_obj[1,3],T_bax_to_obj[2,3])
            
            # plot
            IF_PLOT=True
            if IF_PLOT:
                object_in_image=(xi, yi, radius)
                object_in_chessboard=(p_chess_to_obj[0,0],p_chess_to_obj[1,0],p_chess_to_obj[2,0])

                self.display_object_pose_in_image(self.image_for_display_chessboard,
                    object_in_image=object_in_image,
                    object_in_chessboard=object_in_chessboard)

            # append to list
            poses.append(pose)

        if IF_PLOT:
            self.pub_image_chessboard()

        return GetObjectInBaxterResponse(poses)
    
    def display_object_pose_in_image(self, img_for_display, object_in_image, object_in_chessboard):
        
        (xi, yi, radius)=(object_in_image[0], object_in_image[1], object_in_image[2])

        # ---- Show ----
        # Add circle to img_for_display
        img_for_display = cv2.circle(img_for_display,
                                     center=(int(xi), int(yi)),
                                     radius=int(radius),
                                     color=[0, 0, 255],
                                     thickness=2, lineType=8, shift=0)

        # Add text to img_for_display
        FONT = cv2.FONT_HERSHEY_SIMPLEX
        FONTSIZE = 1
        sss = ["object pos wrt chessboard", "x=", "y="]
        ppp = [xi, yi]
        for i in range(-1, 2):
            if i != -1:
                s = "{:.2f}".format(object_in_chessboard[i])
            else:
                s = ""
            TEST_ROWS = int(yi-80+i*30)
            TEST_COLS = int(xi-50)
            COLOR = 255
            # print sss[i+1]+s, "TEST_COLS=", TEST_COLS, "TEST_ROWS", TEST_ROWS, FONT,\
            #     FONTSIZE, (0, 0, COLOR), 2, cv2.LINE_AA
            img_for_display = cv2.putText(
                img_for_display, sss[i+1]+s, (TEST_COLS, TEST_ROWS), FONT,
                FONTSIZE, (0, 0, COLOR), 2, cv2.LINE_AA)

        # Show image
        # cv2.imshow("img_for_display", img_for_display)
        # cv2.waitKey(1)
        return img_for_display

    def save_image(self, img):
        str_image_idx = "{:05d}".format(self.cnt)
        filename = PACKAGE_PATH+"/images_results/" + \
            IMAGE_FILENAME_FOR_SAVING+'_'+str_image_idx+'.jpg'
        res = cv2.imwrite(filename, img)
        print(res)
        print("    Save image to: %s" % filename)

    def check_if_image_is_valid(self):
        if self.img is None:
            rospy.loginfo(set_str_error("not receiving any image."))
            return False

        if not TEST_MODE:
            if rospy.get_time()-self.t_receive_image>0.5:
                rospy.loginfo(set_str_error("The received image has been long ago."))
                return False

        return True


    def pub_image_chessboard(self):
        self.pub1.publish(
            self.bridge.cv2_to_imgmsg(self.image_for_display_chessboard, "bgr8")
        )

    def pub_image_object(self):
        self.pub2.publish(
            self.bridge.cv2_to_imgmsg(self.image_for_display_object, "bgr8")
        )

def Rp_to_pose(R,p):
    pose=Pose()

    R_vec, _ = cv2.Rodrigues(R)
    q=quaternion_from_euler(R_vec[0],R_vec[1],R_vec[2])    
    pose.orientation.w=q[0]
    pose.orientation.x=q[1]
    pose.orientation.y=q[2]
    pose.orientation.z=q[3]

    pose.position.x=p[0]
    pose.position.y=p[1]
    pose.position.z=p[2]
    return pose

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