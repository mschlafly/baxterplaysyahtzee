#!/usr/bin/env python2
# -*- coding: utf-8 -*-

'''
Provide services for:
1. mycvCalibChessboardPose
2. mycvGetObjectInBaxter
3. mycvGetAllObjectsInBaxter
'''

# ----------------------------------------
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from matplotlib import pyplot as plt
from std_msgs.msg import String
import tf

import os, sys
import time
PACKAGE_PATH = os.path.join(os.path.dirname(__file__))+"/../"
CURRENT_PATH=os.path.join( os.path.dirname(__file__) )+"/"
# sys.path.append(PACKAGE_PATH)
# sys.path.append(PACKAGE_PATH+"")

# ---------------------- Import from our own library -----------------------
from lib_cv_calib import ChessboardLocator, Object3DPoseLocator, find_object, myTrackbar
from lib_cv_detection import refine_image_mask, find_square, extract_rect,\
    find_object_in_middle, find_all_objects, find_all_objects_then_draw, detect_dots, get_color_median
from lib_baxter_camera_config import form_T, get_Rp_from_T


# ---------------------- service provided by this node -----------------------
from tf.transformations import euler_from_quaternion, quaternion_from_euler, euler_matrix
from geometry_msgs.msg import Pose, Point
from baxterplaysyahtzee.msg import ColorBound, ObjectInfo
from baxterplaysyahtzee.srv import GetAllObjectsInImageResponse, GetObjectInImageResponse, \
    GetObjectInBaxterResponse, GetAllObjectsInBaxterResponse, CalibChessboardPoseResponse
from baxterplaysyahtzee.srv import *

# dont forget to call: undistortPoints

# ---------------------- TEST SETTINGS ------------------
TEST_MODE=False
if TEST_MODE:
    DETECT_ONE_OBJECT=True
    if DETECT_ONE_OBJECT:
        TEST_IMAGE_FILENAME=CURRENT_PATH+"/images"+"/imgmid3.png"
    else:
        TEST_IMAGE_FILENAME=CURRENT_PATH+"/images"+"/image3.png"

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

# -------------------

def color_to_string(rgbcolor):
    # we need some algorithm here
    return str(rgbcolor)

# -------------------


# -------------------


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
        self.pub3 = rospy.Publisher("/robot/xdisplay",Image, queue_size=10)

     
        # services 1: calib chessboard (return: Pose)
        s1 = rospy.Service('mycvCalibChessboardPose', CalibChessboardPose, self.srv_CalibChessboardPose)
        self.R_cam_to_chess=None
        self.p_cam_to_chess=None
        self.image_for_display_chessboard=None

        self.flag_calibrated=False
        self.T_bax_to_chess=None # after calibration, fill in this value
        self.tf_listener = tf.TransformListener()

        # services 2: get object in image (return: Point)
        s2 = rospy.Service('mycvGetObjectInImage', GetObjectInImage, self.srv_GetObjectInImage)
        self.object_mask=None

        # services 3: get object in image (return: Point)
        s3 = rospy.Service('mycvGetAllObjectsInImage', GetAllObjectsInImage, self.srv_GetAllObjectsInImage)

        # services 4: get object in Baxter
        s4 = rospy.Service('mycvGetObjectInBaxter', GetObjectInBaxter, self.srv_GetObjectInBaxter)
        self.image_for_display_object=None

        # services 5: get all object in Baxter
        s4 = rospy.Service('mycvGetAllObjectsInBaxter', GetAllObjectsInBaxter, self.srv_GetAllObjectsInBaxter)

    def topic_receive_image_callback(self, rosImage):
        self.img = self.bridge.imgmsg_to_cv2(rosImage, "bgr8")
        self.t_receive_image=rospy.get_time()
        self.cnt+=1

    # tested, OK!!! 
    def srv_CalibChessboardPose(self, req):
        # print("inside the srv_CalibChessboardPose")
        img=self.img.copy()
        if not self.check_if_image_is_valid():
            rospy.loginfo(set_str_error("srv_CalibChessboardPose failed."))
            return

        # print(img.shape)
        # Image processing
        self.chessboard_locator = ChessboardLocator(STR_CAMERA_TYPE, SQUARE_SIZE=SQUARE_SIZE)
        flag, R, p, self.image_for_display_chessboard = self.chessboard_locator.locate_chessboard(
            img, SAVE=False, SHOW=False, PRINT=False)
        # R=np.linalg.inv(R)
        self.pub_image_chessboard()

        # store it 
        (self.R_cam_to_chess, self.p_cam_to_chess)=(R,p)

        # output
        if flag is False:
            print "Fail to calibrate"
            return CalibChessboardPoseResponse(False, Pose())
        else:
            print "Successfully calibrate the chessboard\nR=",R,"\np=",p
            pose=Rp_to_pose(R,p)
            self.flag_calibrated=True

            # transformation matrix
            T_cam_to_chess=form_T(R,p)
            # T_cam_to_chess=np.linalg.inv(form_T(R,p))

            # print "Calibration result: self.get_T_bax_to_cam()", self.get_T_bax_to_cam()
            # print "Calibration result: T_cam_to_chess", T_cam_to_chess
            self.T_bax_to_chess=self.get_T_bax_to_cam().dot(T_cam_to_chess)
            
            # print "Calibration result: T_bax_to_chess", self.T_bax_to_chess

            return CalibChessboardPoseResponse(True, pose)

    # tested, OK!!! 
    def srv_GetAllObjectsInImage(self, req):
        flag, objInfos=self._GetAllObjectsInImage(req)
        return GetAllObjectsInImageResponse(flag, objInfos)
        
    def _GetAllObjectsInImage(self, req):
        print("inside the srv_GetAllObjectsInImage")
        img=self.img.copy()
        rows,cols=img.shape[:2]

        if not self.check_if_image_is_valid():
            rospy.loginfo(set_str_error("srv_GetAllObjectsInImage failed."))
            return

        rects, labeled_img=find_all_objects(img)
        labeled_img = cv2.resize(labeled_img, (0,0), fx=2, fy=2)

        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # -------Remove wrong ones ---
        tmp=list()
        for i in range(len(rects)):
            rect=rects[i]
            (center_x, center_y, radius_x, radius_y, angle)  = extract_rect(rect)
            # Criteria for removing wrong objects
            if center_x<150 or center_x>640-150 or center_y<50 or center_y>400-50:
                continue
            else:
                tmp.append(rect.copy())
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
                    
                # dots
                mask=labeled_img==labeled_img[int(center_y),int(center_x)]
                blank_image = np.zeros(mask.shape, np.uint8)
                blank_image[mask]=1
                mask=blank_image
                rect_int= rect.astype(int)
                # print img.shape, mask.shape, rect_int
                # print "rect: ", rect
                # print "rect_int: ", rect_int
                ndots=detect_dots(img, mask, rect_int) # detect dots
                objInfo.value=ndots

                # color
                rgbcolor=get_color_median(img, mask, rect_int)
                objInfo.color=color_to_string(rgbcolor)

                # xyz pose
                objInfo.index=i
                (objInfo.xi, objInfo.yi, objInfo.radius_x, objInfo.radius_y, objInfo.angle)=\
                    (center_x, center_y, radius_x, radius_y, angle)
                objInfo.radius_mean=(radius_x+radius_y)/2

                objInfos.append(objInfo)
            
            objInfos.sort(key=lambda x: x.radius_mean, reverse=True)
            return True, objInfos
        else:
            return False, objInfos


    # tested, OK!!! 
    def srv_GetObjectInImage(self, req):
        flag, objInfo = self._GetObjectInImage(req)
        return GetObjectInImageResponse(flag, objInfo)

    def _GetObjectInImage(self, req):
        img=self.img.copy()
        if not self.check_if_image_is_valid():
            rospy.loginfo(set_str_error("srv_CalibChessboardPose failed."))
            assert(0)

        # Detect object in the image
        mask, rect = find_object_in_middle(img, ratio_RADIUS_TO_CHECK=3, disextend=50)

        # Display image      
        if rect is not None:  
            self.image_for_display_object=cv2.drawContours(img, [rect], 0, [0,1,1], 2)
            self.pub_image_object()

            # output:
            objInfo=ObjectInfo()
            
            # color and dots
            ndots=detect_dots(img, mask, rect) # detect dots
            rgbcolor=get_color_median(img, mask, rect)
            objInfo.value=ndots
            objInfo.color=color_to_string(rgbcolor)

            # save vars
            self.object_mask=mask
            (center_x, center_y, radius_x, radius_y, angle)  = extract_rect(rect)


            (objInfo.xi, objInfo.yi, objInfo.radius_x, objInfo.radius_y, objInfo.angle)=\
                (center_x, center_y, radius_x, radius_y, angle)
            objInfo.radius_mean=(radius_x+radius_y)/2
            return True, objInfo
        else:
            objInfo=ObjectInfo()
            return False, objInfo

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

    def set_up_test_mode_for_GetObjectInBaxter(self):
        if self.R_cam_to_chess is None: # Not calibrated?
            self.srv_CalibChessboardPose(None)
        if self.R_cam_to_chess is None:
            rospy.loginfo(set_str_error("still wrong? I'll manually set it... Now it's Ok "))
            R_vec=[-0.15,0.14,0.26]
            p=[2.23,-7.44,27.02]
            R_vec=np.array([R_vec])
            p=np.array(p)*0.015
            R, _ = cv2.Rodrigues(R_vec)
            self.R_cam_to_chess=R
            self.p_cam_to_chess=p # both 3-vec or 3x1-mat would be fine
        R_cam_to_chess=self.R_cam_to_chess
        p_cam_to_chess=self.p_cam_to_chess
        T_bax_to_cam=np.identity(4)
        return R_cam_to_chess, p_cam_to_chess, T_bax_to_cam

    def srv_GetObjectInBaxter(self, req):
        img=self.img.copy()
        self.image_for_display_chessboard=img.copy() 
        self.image_for_display_object=img.copy()
        if TEST_MODE: # Extract data from the stored chessboard pos
            R_cam_to_chess, p_cam_to_chess, T_bax_to_cam=\
                self.set_up_test_mode_for_GetObjectInBaxter()
        else:
            R_cam_to_chess, p_cam_to_chess, T_bax_to_cam=self.get_required_poses_for_2d3d()
            
        poseLocator = Object3DPoseLocator( # initialize
            STR_CAMERA_TYPE,
            R_cam_table=R_cam_to_chess,
            p_cam_table=p_cam_to_chess
        )

        # Detect objects
        flag, objInfo=self._GetObjectInImage(None)
        print "detecting result: ", flag, objInfo

        if flag == False:
            self.pub_to_screen()
            return GetObjectInBaxterResponse(False, objInfo    )
        else:
            IF_DISPLAY_IMAGE=True
            pose = self.calc_object_pose_in_baxter_frame(
                objInfo, poseLocator, T_bax_to_cam,
                IF_DISPLAY_IMAGE=IF_DISPLAY_IMAGE)

            objInfo.pose=pose
            
            if IF_DISPLAY_IMAGE:
                self.pub_image_chessboard()
                self.pub_to_screen()
            return GetObjectInBaxterResponse(True, objInfo)
    
    def srv_GetAllObjectsInBaxter(self, req):
        img=self.img.copy()
        self.image_for_display_chessboard=img.copy() 
        self.image_for_display_object=img.copy()
        if TEST_MODE: # Extract data from the stored chessboard pos
            R_cam_to_chess, p_cam_to_chess, T_bax_to_cam=\
                self.set_up_test_mode_for_GetObjectInBaxter()
        else:
            R_cam_to_chess, p_cam_to_chess, T_bax_to_cam=self.get_required_poses_for_2d3d()
          
        poseLocator = Object3DPoseLocator( # initialize
            STR_CAMERA_TYPE,
            R_cam_table=R_cam_to_chess,
            p_cam_table=p_cam_to_chess
        )

        # Detect objects
        flag, objInfos=self._GetAllObjectsInImage(None)

        if flag == False:
            self.pub_to_screen()
            return GetAllObjectsInBaxterResponse(False, list())

        else:
            IF_DISPLAY_IMAGE=True
            # calc the pose of each object
            for i in range(len(objInfos)):
                pose = self.calc_object_pose_in_baxter_frame(
                    objInfos[i], poseLocator, T_bax_to_cam,
                    IF_DISPLAY_IMAGE=IF_DISPLAY_IMAGE)
                objInfos[i].pose=pose
                
            if IF_DISPLAY_IMAGE:
                self.pub_image_chessboard()
                self.pub_to_screen()
            return GetAllObjectsInBaxterResponse(True, objInfos)

    def calc_object_pose_in_baxter_frame(self, objInfo, poseLocator, T_bax_to_cam,
            IF_DISPLAY_IMAGE=True
        ):
        
        (xi, yi, radius_x, radius_y, angle) = \
            (objInfo.xi, objInfo.yi, objInfo.radius_x, objInfo.radius_y, objInfo.angle)
        radius=objInfo.radius_mean

        # Locate the object 3D (x,y,z) wrt camera frame and chessboard frame
        p_cam_to_obj, p_chess_to_obj = \
            poseLocator.locate_object(xi=xi, yi=yi, PRINT=False) # format: (3,1) column vector
        # print "p_cam_to_obj:\n",p_cam_to_obj
        # print "p_chess_to_obj:\n",p_chess_to_obj

        # Locate the objects direction
        # Input: self.object_mask
        T_cam_to_obj=form_T(R=np.identity(3),p=p_cam_to_obj)

        # transform p_in_camera to p_in_world
        T_bax_to_obj=T_bax_to_cam.dot(T_cam_to_obj)            

        # plot
        if IF_DISPLAY_IMAGE:
            object_in_image=(xi, yi, radius)
            object_in_chessboard=(p_chess_to_obj[0,0],p_chess_to_obj[1,0],p_chess_to_obj[2,0])

            self.display_object_pose_in_image(self.image_for_display_chessboard,
                object_in_image=object_in_image,
                radius=radius,
                object_in_chessboard=object_in_chessboard,
                dice_value=objInfo.value)

        # return
        pose=Pose()
        (pose.position.x, pose.position.y, pose.position.z)=\
            (T_bax_to_obj[0,3],T_bax_to_obj[1,3],T_bax_to_obj[2,3])

        return pose

    def display_object_pose_in_image(self, img_for_display, object_in_image, radius, 
            object_in_chessboard, dice_value):
        
        (xi, yi)=(object_in_image[0], object_in_image[1])

        # ---- Show ----
        # Add circle to img_for_display
        img_for_display = cv2.circle(img_for_display,
                                     center=(int(xi), int(yi)),
                                     radius=int(radius),
                                     color=[0, 0, 255],
                                     thickness=2, lineType=8, shift=0)

        # Add text to img_for_display
        FONT = cv2.FONT_HERSHEY_SIMPLEX
        FONTSIZE = 1.0
        sss = ["pos", "x=", "y=","dice="]
        ppp = [xi, yi]
        for i in range(-1, 3):
            if i ==0 or i==1:
                s = "{:.2f}".format(object_in_chessboard[i])
            elif i==2:
                print "dice_value in 2",dice_value
                s = str(dice_value)
            else:
                s = ""
            TEST_ROWS = int(yi+i*30)
            TEST_COLS = int(xi)
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

    def pub_to_screen(self):
        if self.image_for_display_object is not None and self.image_for_display_chessboard is not None:
            # tmpimage=np.hstack([self.image_for_display_object,self.image_for_display_chessboard])
            tmpimage=np.hstack([self.image_for_display_object,self.image_for_display_chessboard])
            tmpimage=cv2.resize(tmpimage,(0,0),fx=0.8,fy=0.8) #!!!!!!!!!
            self.pub3.publish(self.bridge.cv2_to_imgmsg(tmpimage, "bgr8"))

    # ---------------------------------------------
    def get_T_bax_to_cam(self):
        (trans, rot) = self.tf_listener.lookupTransform(
            '/base', '/left_hand_camera', rospy.Time(0))
        # print "quaternion from tf = ", rot
        r3=euler_from_quaternion(rot)
        # print "euler_matrix=",euler_matrix(r3[0],r3[1],r3[2])
        R=euler_matrix(r3[0],r3[1],r3[2])[0:3,0:3]
        # R,_=cv2.Rodrigues(r3)
        # R=np.linalg.inv(R)
        T = form_T(R,trans)
        return T

    def get_required_poses_for_2d3d(self):
        T_bax_to_cam=self.get_T_bax_to_cam()
        T_cam_to_bax=np.linalg.inv(T_bax_to_cam)
        # print "T_cam_to_bax",T_cam_to_bax
        # print "T_bax_to_cam",T_bax_to_cam
        T_cam_to_chess = T_cam_to_bax.dot(self.T_bax_to_chess)
        R_cam_to_chess, p_cam_to_chess =get_Rp_from_T(T_cam_to_chess)

        # T_bax_to_cam: correct
        # p_cam_to_chess: wrong
        # print R_cam_to_chess, p_cam_to_chess, T_bax_to_cam
        return R_cam_to_chess, p_cam_to_chess, T_bax_to_cam

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

# def callback_ColorBound(msg):
#     global COLOR_LB, COLOR_UB
#     COLOR_LB=(msg.low_bound0,msg.low_bound1,msg.low_bound2)
#     COLOR_UB=(msg.high_bound0, msg.high_bound1, msg.high_bound2)

if __name__ == '__main__':
    rospy.init_node('nodeCV')
    # sub=rospy.Subscriber("ColorBound",ColorBound,callback_ColorBound)
    BaxterCameraProcessing()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
