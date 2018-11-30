#!/usr/bin/env python2
# -*- coding: utf-8 -*-

# Detect chessboard in image, then calculate it's coords in camera's coords.
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
from baxter_camera_config_lib import form_T, get_Rp_from_T
from baxter_camera_config_lib import BaxterCamera_Head, BaxterCamera_LeftHand, BaxterCamera_RightHand

cameras = [BaxterCamera_Head(), 
           BaxterCamera_LeftHand(), 
           BaxterCamera_RightHand()]

CAMERA = cameras[0] 
LEN_PER_SQUARE = 0.015
CHECKER_ROWS = 7
CHECKER_COLS = 9
SHOW = False
TEST_MULTIPLE_IMAGES = True

objp = np.zeros((CHECKER_COLS*CHECKER_ROWS, 3), np.float32)*LEN_PER_SQUARE
objp[:, :2] = np.mgrid[0:CHECKER_ROWS, 0:CHECKER_COLS].T.reshape(-1, 2)

# set input images
if TEST_MULTIPLE_IMAGES:  # input all images under a folder
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

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space

# start
cnt_img = 0
for fname in images:
    cnt_img += 1
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    # ret, corners = cv2.findChessboardCorners(gray, (CHECKER_ROWS, CHECKER_COLS), None)
    ret, corners = cv2.findChessboardCorners(img, (CHECKER_ROWS, CHECKER_COLS), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        print("chessboard found, img = {}.".format(fname))
        # save data

        # corners pos in image
        refined_corners = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1), criteria) # refine position

        # solve PnP: transformation of chessboard wrt camera
        objpoint0=objp
        imgpoint0=refined_corners
        cameraMatrix=CAMERA.intrinsic_matrix
        distCoeffs=CAMERA.distortion

        # retval0, rotation_vec0, translation_vec0 = cv2.solvePnP(objpoint0, imgpoint0, cameraMatrix, distCoeffs)
        retval0, rotation_vec0, translation_vec0, inliers = cv2.solvePnPRansac(objpoint0, imgpoint0, cameraMatrix, distCoeffs)
        R, _ = cv2.Rodrigues(rotation_vec0)
        p = translation_vec0

        print("chessboard position in camera frame is:")
        print p

        # Draw chessboard
        img = cv2.drawChessboardCorners(
            img, (CHECKER_ROWS, CHECKER_COLS), refined_corners, ret
        )
        # Add text
        font = cv2.FONT_HERSHEY_SIMPLEX
        sss=["x=","y=","z="]
        for i in range(3):
          s="{:.2f}".format(p[i,0])
          column=200+i*30
          img = cv2.putText(img,sss[i]+s,(30,column), font, 1,(255,255,255),2,cv2.LINE_AA)
        
        # show
        if SHOW:
          cv2.imshow('img', img)

        # save file
        str_index = "{:03d}".format(cnt_img)
        cv2.imwrite(str_folder + "_" + str_index + '.png', img)
    else:
        print("chessboard not found, img = {}.".format(fname))
        if SHOW:
          cv2.imshow('img', img)
    cv2.waitKey(1)

cv2.waitKey()
cv2.destroyAllWindows()
