#!/usr/bin/env python2
# -*- coding: utf-8 -*-

# Calibrate camera by using images in the ./original_images,
#       which are from Baxter's head camera.
# The chessboard is displayed on Feiyu Chen's laptop.
#       Displayed by image viewer, 67% resize. Words in the image are towards left.
#       Square's size in real world: 0.017 meters
#       (Values on diagonal of the camera intrinsic matrix should be multiplied by this value)
# CHESSBOARD_SQUARE_SIZE=0.017

# The calibration of about 50 images take about 3 minutes.

import numpy as np
import cv2
import glob # for getting files' names in the disk
import pickle # for saving variables to disk

CAMERA_TYPES=["head","lefthand","righthand"]
CAMERA_TYPE=CAMERA_TYPES[0]
FOLDER_OF_ORIGINAL_IMAGES=CAMERA_TYPE+"_original_images"
CAMERA_PARAM_OUTPUT_FILENAME=CAMERA_TYPE+'_camera_parameters.pkl'

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....
CHECKER_COLS=9
CHECKER_ROWS=7
objp = np.zeros(( CHECKER_COLS* CHECKER_ROWS,3), np.float32)
objp[:,:2] = np.mgrid[0: CHECKER_ROWS,0: CHECKER_COLS].T.reshape(-1,2)
# objp=objp*CHESSBOARD_SQUARE_SIZE

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob(FOLDER_OF_ORIGINAL_IMAGES+'/*.jpg')

cnt_img=0
for fname in images:
    cnt_img+=1
    print("processing {}/{} image ...".format(cnt_img, len(images)))
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, ( CHECKER_ROWS, CHECKER_COLS),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        
        objpoints.append(objp)
    
        corners_refined = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners_refined)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, ( CHECKER_ROWS, CHECKER_COLS), corners_refined,ret)
        cv2.imshow('img',img)
        # cv2.imwrite('img'+str(cnt_img)+'.png',img)
        cv2.waitKey(1)

cv2.destroyAllWindows()

# Calibrate camera
projection_error, camera_intrinsics, camera_distortion_coefs, rvecs, tvecs = \
    cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
print projection_error # What is the standard of this value? 
print camera_intrinsics
print camera_distortion_coefs

# Saving the objects:
with open(CAMERA_PARAM_OUTPUT_FILENAME, 'w') as f: 
    pickle.dump([projection_error, camera_intrinsics, camera_distortion_coefs], f)

# # Getting back the objects:
# with open(CAMERA_PARAM_OUTPUT_FILENAME) as f:  # Python 3: open(..., 'rb')
#     obj0, obj1, obj2 = pickle.load(f)