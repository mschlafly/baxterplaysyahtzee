


import numpy as np
import cv2
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER + cv2.CALIB_CB_FAST_CHECK, 30, 0.001) # termination criteria
import glob
import sympy as sp

import sys, os
PROJECT_PATH=os.path.join(os.path.dirname(__file__))+ "/../"

# INCLUDE_PATHS=["config",]
# for i in range(len(INCLUDE_PATHS)):
#     sys.path.append(PROJECT_PATH+INCLUDE_PATHS[0])

# import our libraries
from baxter_camera_config_lib import BaxterCamera_Head, BaxterCamera_LeftHand, BaxterCamera_RightHand
from ourlib_transformations import form_T, get_Rp_from_T
import ourlib_transformations as trans

def choose_baxter_camera(STR_CAMERA_TYPE):
    if STR_CAMERA_TYPE=="Head":
        camera=BaxterCamera_Head()
    elif STR_CAMERA_TYPE=="Left":
        camera=BaxterCamera_LeftHand()
    elif STR_CAMERA_TYPE=="Right":
        camera=BaxterCamera_RightHand()
    else:
        print("Wrong camera type")
        assert(0)
    return camera

class ChessboardLocator(object):
    # Detect chessboard in image, and then calculate it's coords in self.CAMERA's coords.
    # You should set:
    #       * folder where image/images are stored
    #       * which baxter self.CAMERA to use (because we need self.CAMERA info)
    #       * length of the squares in the chessboard
    # then, this script will save the resultant images, with all notations added.
    def __init__(self,
            STR_CAMERA_TYPE="Left",
            SQUARE_SIZE=0.015, # needs to measure by a ruler
        ):
        self.CAMERA=choose_baxter_camera(STR_CAMERA_TYPE)

        # Set length of the squares in the chessboard
        self.SQUARE_SIZE = SQUARE_SIZE

        # Chessboard squares number
        self.CHECKER_ROWS = 5
        self.CHECKER_COLS = 6
        # self.CHECKER_ROWS = 7
        # self.CHECKER_COLS = 9

    # Points coodinate in the chessboard frame
    # (whose technique term is "object points". Opposite to "image points")
    def create_object_points(self, square_size=None):
        square_size = self.square_size if square_size is None else square_size
        objpoint = np.zeros((self.CHECKER_COLS*self.CHECKER_ROWS, 3), np.float32)
        objpoint[:, :2] = np.mgrid[0:self.CHECKER_ROWS, 0:self.CHECKER_COLS].T.reshape(-1, 2)*square_size
        return objpoint

    def locate_chessboard(self, img,
            SHOW=False, SHOW_TIME_MS=5, SAVE=False, PRINT=True,
            OUTPUT_FOLDER = "/",
            OUTPUT_IMAGE_FILENAME="image_from_ChessboardLocator.jpg",
        ):
        # check input
        OUTPUT_IMAGE_FILENAME="res_"+OUTPUT_IMAGE_FILENAME.split("/")[-1]
        
        # image rgb to gray
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        flag_find_chessboard, corners = cv2.findChessboardCorners(
            gray, (self.CHECKER_ROWS, self.CHECKER_COLS), None) 

        # Coordinate values of corners in chessboard frame
        objpoint=self.create_object_points(square_size=self.SQUARE_SIZE)

        # If found, add object points, image points (after refining them)
        if flag_find_chessboard == True:
            if PRINT:
                print("chessboard found, file = {}.".format(OUTPUT_IMAGE_FILENAME))

            # corners pos in image
            refined_corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria)  # refine position

            # solve PnP: transformation of chessboard wrt self.CAMERA
            objpoint = objpoint
            imgpoint = refined_corners
            # print "\n\nobject",object
            # print "imgpoint",imgpoint
            camera_intrinsics = self.CAMERA.intrinsic_matrix
            distortion_coeffs = self.CAMERA.distortion

            # note: or use cv2.solvePnP (same input; one less output of "inliers")
            err_value, R_vec, p, inliers = cv2.solvePnPRansac(
                objpoint, imgpoint, camera_intrinsics, distortion_coeffs)
            R, _ = cv2.Rodrigues(R_vec)
            
            # T=form_T(R,p)
            # T=np.linalg.inv(T)
            # R,p=get_Rp_from_T(T)
            # R_vec,_=cv2.Rodrigues(R)

            if True or PRINT:    
                print("chessboard position in camera frame is:")
                print "  R_vec={:.2f},{:.2f},{:.2f}\n  p={:.2f},{:.2f},{:.2f}".format(
                    R_vec[0,0],R_vec[1,0],R_vec[2,0], p[0,0], p[1,0], p[2,0]
                )

            # Draw chessboard
            img = cv2.drawChessboardCorners(
                img, (self.CHECKER_ROWS, self.CHECKER_COLS), imgpoint, flag_find_chessboard
            )
            # Add text
            font = cv2.FONT_HERSHEY_SIMPLEX
            sss = ["chessboard pose wrt to camera","x=", "y=", "z="]
            for i in range(-1,3):
                if i!=-1:
                    s = "{:.2f}".format(p[i, 0])
                else:
                    s=""
                TEST_ROWS = 50+i*30
                TEST_COLS = 50
                img = cv2.putText(
                    img, sss[i+1]+s, (TEST_COLS, TEST_ROWS), font, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

            # show
            if SHOW:
                cv2.imshow('img', img)
            if SAVE:
                # save file
                OUTPUT_INDEX=1
                OUTPUT_INDEX_STR = "{:04d}".format(OUTPUT_INDEX)
                cv2.imwrite(PROJECT_PATH+OUTPUT_FOLDER+OUTPUT_IMAGE_FILENAME +
                            "_" + OUTPUT_INDEX_STR + '.png', img)
            img_for_display = img
        else: # flag_find_chessboard == False
            if PRINT:    
                print("chessboard not found, img = {}.".format(OUTPUT_IMAGE_FILENAME))
            if SHOW:
                cv2.imshow('img', img)
            R = None
            p = None
            img_for_display = img

        if SHOW:
            cv2.waitKey(int(SHOW_TIME_MS))
            cv2.destroyAllWindows()

        return flag_find_chessboard, R, p, img_for_display

class Object3DPoseLocator(object):
    # After we know
    #   1. where the table is wrt camera,
    #   2. object is on the table
    #   3. object's pixel in the image
    # We know the object's position wrt camera

    def __init__(self,
            STR_CAMERA_TYPE="Left",
            # Input either {R,p} or {T}, 
            #   which is the transformation from camera to table
            R_cam_table=None, 
            p_cam_table=None,
            T_cam_table=None, 
        ):

        # check input
        self.CAMERA=choose_baxter_camera(STR_CAMERA_TYPE)
        if T_cam_table is None:
            T_cam_table=trans.form_T(R=R_cam_table, p=p_cam_table)
        self.T_cam_table=T_cam_table
        self.K=self.CAMERA.intrinsic_matrix

    def locate_object(self,
            xi, # x pixel of the object in the image
            yi, # y pixel of the object in the image
            PRINT=False,
        ):
        '''
        principles and formulas={
            camera_intrinsics=[
                    fx, 0,  cx
                    0 , fy, cy
                    0 ,  0,  1
                    ]
            
            fx: focal length in pixel along x direction
            fy: focal length in pixel along y direction
            cx: x coord of camera axis in image
            cy: y coord of camera axis in image
            
            xi: x coord of object in image
            yi: y coord of object in image
            
            Position of the object wrt camera
                == scale_factor * [xi, yi, 1].T
                == camera_intrinsics * T_camera_board * P_board
            
            P_board: coord of object in the chessboard
                == [Px, Py, Pz] == [Px, Py, 0]
                
            Since we know the object is on table, we get Pz == 0.
            Solving the equation, we get the scale_factor,
            which indicates the depth and the position of the object wrt camera.
            (note: Px and Py is not important)
        }
        '''
        P_image=np.array([[xi,yi,1]]).T
        
        # P_board: object pos in chessboard frame
        Px, Py=sp.symbols('Px Py')
        P_board=np.array([[Px,Py,0,1]]).T

        # P_camera: object pos in camera frame
        T=self.T_cam_table
        P_camera=T.dot(P_board)[0:3,0:1]
        
        # Equation
        s=sp.symbols('s')
        eq = s*P_image-self.K.dot(P_camera)
        
        # Solve
        res = sp.solve([eq[0,0],eq[1,0],eq[2,0]], s, Px, Py)

        if len(res)==0:
            print("Solution not found")
            assert(0)
        vs=res[s]
        vPx=res[Px]
        vPy=res[Py]
        
        res_P_board=np.array([[vPx,vPy,0,1]]).T
        res_P_camera=T.dot(res_P_board)[0:3,0:1]
        res_P_board=res_P_board[0:3,0:1]
        
        if PRINT:
            print "\nObject pos in camera frame:\n",res_P_camera.T
            print "\nObject pos in chessboard frame:\n",res_P_board.T
        return res_P_camera, res_P_board
    
    def _test(self):# This is just a test case.
        # The tested image is https://drive.google.com/file/d/1sH6mXxbCuN_FGoVtnhtznShyth0Cy0n7/view?usp=sharing
        '''
        run codes below:
            ol=ObjectLocator(T_cam_table=np.identity(4))
            ol._test()
        '''
        R_vec=[-0.15,0.14,0.26]
        p=[2.23,-7.44,27.02]
        
        R_vec=np.array([R_vec])
        p=np.array(p)*0.015
        
        R, _ = cv2.Rodrigues(R_vec)
        T=trans.form_T(R,p)
        
        intrinsic_matrix_array_form=[
                403.1498765559809, 0.0, 651.3100088479307/2,
                0.0, 403.9409358133219, 405.82095914289783/2,
                0.0, 0.0, 1.0]
        intrinsic_matrix=np.reshape(intrinsic_matrix_array_form, (3,3))
        self.K=intrinsic_matrix
        self.T_cam_table=T
        self.locate_object(xi=500, yi=500, PRINT=True)

class myTrackbar(object):

    def __init__(self, window_name):
        cv2.namedWindow(window_name)
        # create trackbars for color change
        LB=(35, 43, 46)
        UB=(77, 255, 255)
        self.window_name=window_name
        self.create_bar()
        self.set_bar(LB,UB)

    def nothing(self, x):
        return

    def create_bar(self):
        cv2.createTrackbar('h_low',self.window_name,0,255,self.nothing)
        cv2.createTrackbar('h_high',self.window_name,0,255,self.nothing)
        cv2.createTrackbar('s_low',self.window_name,0,255,self.nothing)
        cv2.createTrackbar('s_high',self.window_name,0,255,self.nothing)
        cv2.createTrackbar('v_low',self.window_name,0,255,self.nothing)
        cv2.createTrackbar('v_high',self.window_name,0,255,self.nothing)
        
    def set_bar(self,LB,UB):
        cv2.setTrackbarPos('h_high',self.window_name,UB[0])
        cv2.setTrackbarPos('s_high',self.window_name,UB[1])
        cv2.setTrackbarPos('v_high',self.window_name,UB[2])
        cv2.setTrackbarPos('h_low',self.window_name,LB[0])
        cv2.setTrackbarPos('s_low',self.window_name,LB[1])
        cv2.setTrackbarPos('v_low',self.window_name,LB[2])
        
    def get_trackbar_values_LB_UB(self):
        h_low = cv2.getTrackbarPos('h_low',self.window_name)
        h_high = cv2.getTrackbarPos('h_high',self.window_name)
        s_low = cv2.getTrackbarPos('s_low',self.window_name)
        s_high = cv2.getTrackbarPos('s_high',self.window_name)
        v_low = cv2.getTrackbarPos('v_low',self.window_name)
        v_high = cv2.getTrackbarPos('v_high',self.window_name)
        return (h_low,s_low,v_low),(h_high,s_high,v_high)

def find_object(img,
            LB = (29, 86, 6),
            UB = (64, 255, 255)
        ):

        erode_iterations=2

        # (rows,cols,channels) = img.shape
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(hsv,LB,UB)
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.erode(mask, kernel, erode_iterations)
        mask = cv2.dilate(mask, kernel, erode_iterations+3)
    
        #	# find contours in the mask and initialize the current
        #	# (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        #    # cnts = cnts[0] if imutils.is_cv2() else cnts[1]
        cnts = cnts[1]
        center = None
        mask = cv2.cvtColor(mask,cv2.COLOR_GRAY2RGB)
        
    	# only proceed if at least one contour was found
        if len(cnts) > 0:
    		# find the largest contour in the mask, then use
    		# it to compute the minimum enclosing circle and
    		# centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            radius=int(radius)
            if radius >= 10: # only proceed if the radius meets a minimum size
                # M = cv2.moments(c)
                # center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                # draw the circle and centroid on the img,
                # then update the list of tracked points
                center=(int(x),int(y))
                cv2.circle(img, center, radius,(0, 255, 255), 2)
                cv2.circle(img, center, 1, (0, 0, 255), -1)
        if center is None:
            return None, None, None, mask
        else:
            return center[0],center[1],int(radius),mask

def find_object2(img): # This is the more advanced method
    
    None