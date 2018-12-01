#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import numpy as np
import cv2
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001) # termination criteria
import glob

import sys, os
PROJECT_PATH=os.path.join(os.path.dirname(__file__))+ "/../"

# INCLUDE_PATHS=["config",]
# for i in range(len(INCLUDE_PATHS)):
#     sys.path.append(PROJECT_PATH+INCLUDE_PATHS[0])

# ------------------------------------------------------------------------------------

from ourlib_cv import ChessboardLocator, Object3DPoseLocator

HEAD_CAMERA=False
if HEAD_CAMERA:
    IMAGE_FILENAME0="baxter_head_camera_image.jpg"
    STR_CAMERA_TYPE="Head"
    X_IN_IMAGE=750 # object pos in image
    Y_IN_IMAGE=750
else:
    IMAGE_FILENAME0="baxter_left_hand_camera_image.jpg"
    STR_CAMERA_TYPE="Left"
    X_IN_IMAGE=300 # object pos in image
    Y_IN_IMAGE=200
IMAGE_FOLDERNAME="./test_cv/"
SQUARE_SIZE=0.02

def main():
    IMAGE_FILENAME=PROJECT_PATH+IMAGE_FOLDERNAME+IMAGE_FILENAME0
    OUTPUT_NAME=PROJECT_PATH+IMAGE_FOLDERNAME+"out_"+IMAGE_FILENAME0

    img = cv2.imread(IMAGE_FILENAME)

    # detect chessboard
    cl=ChessboardLocator(STR_CAMERA_TYPE,SQUARE_SIZE=SQUARE_SIZE)
    flag, R, p, img_annotated = cl.locate_chessboard(
        img,
        OUTPUT_IMAGE_FILENAME=IMAGE_FILENAME,
        SAVE=False
    )
    if flag is False:
        print("chessboard not found")
        assert(0)
    # Detect object in the image
    # Suppose we already know where the object is.
    xi=X_IN_IMAGE
    yi=Y_IN_IMAGE

    # Locate the object 3D pose wrt camera frame and chessboard frame
    op=Object3DPoseLocator(
        STR_CAMERA_TYPE,
        R_cam_table=R,
        p_cam_table=p,
    )
    res_P_camera, res_P_board=op.locate_object(xi=xi, yi=yi,PRINT=True)

    # ---- Show ----
    # Add circle
    img_annotated=cv2.circle(img_annotated, 
        center=(xi, yi),
        radius=5,
        color=[0, 0, 255],
        thickness=10, lineType=8, shift=0)

    # Add text
    font = cv2.FONT_HERSHEY_SIMPLEX
    sss = ["object pos wrt chessboard","x=", "y="]
    ppp = [xi, yi]
    for i in range(-1,2):
        if i!=-1:
            s = "{:.2f}".format(res_P_board[i,0])
        else:
            s = ""
        TEST_ROWS = Y_IN_IMAGE-80+i*30
        TEST_COLS = X_IN_IMAGE-50
        COLOR=255
        img_annotated = cv2.putText(
                    img_annotated, sss[i+1]+s, (TEST_COLS, TEST_ROWS), font, 
                    0.8, (0, 0, COLOR), 2, cv2.LINE_AA)

    cv2.imwrite(OUTPUT_NAME, img_annotated)
    cv2.imshow("object show in red dot", img_annotated)
    cv2.waitKey()
    cv2.destroyAllWindows()

if __name__=="__main__":
    main()
