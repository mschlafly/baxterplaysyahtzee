
#!/usr/bin/env python2
# -*- coding: utf-8 -*-

# Detect the chessboard in the image, and then calculate its coodinate in the camera's coordinate.
# You should set:
#       * folder where images are stored
#       * which baxter camera to use (because we need camera info)
# then, this script will save the resultant images, with all notations added.


import numpy as np
import cv2
import glob

# import sys, os
# PROJECT_PATH=os.path.join(os.path.dirname(__file__))+ "/../"
# sys.path.append(PROJECT_PATH)

# load robot model
from lib import form_T, get_Rp_from_T
from lib import BaxterCamera_Head, BaxterCamera_LeftHand

baxter_camera_head=BaxterCamera_Head()
baxter_camera_left_hand=BaxterCamera_LeftHand()

# choose a camera
camera = baxter_camera_left_hand
len_per_square=0.015
if_show=False

# set input images
if 1:  # input all images under a folder
    str_folders = [
      "images_baxter_head",
      "images_baxter_head_scene2",
      "images_baxter_left_hand",
      "newscripts"]

    str_folder = str_folders[2]
    images = glob.glob("./"+str_folder+"/*.jpg")
else:
    str_folder = "newscripts"
    images = ["./"+str_folder+"/baxter_image_100.jpg", ]
print(images)



# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(num_cols-1,num_rows-1,0)
num_rows = 7
num_cols = 9
objp = np.zeros((num_cols*num_rows, 3), np.float32)*len_per_square
objp[:, :2] = np.mgrid[0:num_rows, 0:num_cols].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.

# start
cnt_img = 0
for fname in images:
    cnt_img += 1
    I = cv2.imread(fname)
    gray = cv2.cvtColor(I, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    # ret, corners = cv2.findChessboardCorners(gray, (num_rows, num_cols), None)
    ret, corners = cv2.findChessboardCorners(I, (num_rows, num_cols), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        print("chessboard found, I = {}.".format(fname))
        # save data

        # corners pos in real world,
        objpoints.append(objp)

        # corners pos in image
        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1), criteria) # refine position
        imgpoints.append(corners2)

        # solve PnP: transformation of chessboard wrt camera
        objpoint0=objp
        imgpoint0=corners2
        cameraMatrix=camera.intrinsic_matrix
        distCoeffs=camera.distortion

        # retval0, rotation_vec0, translation_vec0 = cv2.solvePnP(objpoint0, imgpoint0, cameraMatrix, distCoeffs)
        retval0, rotation_vec0, translation_vec0, inliers = cv2.solvePnPRansac(objpoint0, imgpoint0, cameraMatrix, distCoeffs)
        R, _ = cv2.Rodrigues(rotation_vec0)
        p = translation_vec0

        # the above transformation is from object to camera,
        # we need to inverse it
        if 0:
            T_inv=form_T(R,p)
            T=np.linalg.inv(T_inv)
            R, p=get_Rp_from_T(T) # R:rot, p: pose

        print("chessboard position in camera frame is:")
        print p

        # Draw chessboard
        I = cv2.drawChessboardCorners(
            I, (num_rows, num_cols), corners2, ret
        )
        # Add text
        font = cv2.FONT_HERSHEY_SIMPLEX
        sss=["x=","y=","z="]
        for i in range(3):
          s="{:.2f}".format(p[i,0])
          column=200+i*30
          I = cv2.putText(I,sss[i]+s,(30,column), font, 1,(255,255,255),2,cv2.LINE_AA)
        
        # show
        if if_show:
          cv2.imshow('I', I)

        # save file
        str_index = "{:03d}".format(cnt_img)
        cv2.imwrite(str_folder + "_" + str_index + '.png', I)
    else:
        print("chessboard not found, I = {}.".format(fname))
        if if_show:
          cv2.imshow('I', I)
    cv2.waitKey(1)
      

cv2.waitKey()
cv2.destroyAllWindows()


'''
# calibration
ret, mtx, dist, rvecs, tvecs = \
    cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

ret:error
mtx: inner matrix
dist 1x5: coef?
rvecs:rotation?
tvecs:translation?
    
'''
