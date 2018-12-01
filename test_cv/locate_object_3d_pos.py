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

IMAGE_FOLDERNAME="./test_cv/"
IMAGE_FILENAME0="baxter_left_hand_camera_image.jpg"

STR_CAMERA_TYPE="Left"
SQUARE_SIZE=0.02

def main():
    IMAGE_FILENAME=PROJECT_PATH+IMAGE_FOLDERNAME+IMAGE_FILENAME0
    img = cv2.imread(IMAGE_FILENAME)

    # detect chessboard
    cl=ChessboardLocator(STR_CAMERA_TYPE,SQUARE_SIZE=SQUARE_SIZE)
    flag, R, p, img_annotated = cl.locate_chessboard(
        img,
        OUTPUT_IMAGE_FILENAME=IMAGE_FILENAME
    )

    # Detect object in the image
    # Suppose we already know where the object is.
    x_in_image=500
    y_in_image=300

    # Locate the object 3D pose wrt camera frame and chessboard frame
    op=Object3DPoseLocator(
        STR_CAMERA_TYPE,
        R_cam_table=R,
        p_cam_table=p
    )
    res_P_camera, res_P_board=op.locate_object(xi=x_in_image, yi=y_in_image,PRINT=True)

    # ---- Show ----
    # Add circle
    img_annotated=cv2.circle(img_annotated, 
        center=(x_in_image, y_in_image),
        radius=5,
        color=[0, 0, 255],
        thickness=10, lineType=8, shift=0)

    # Add text
    font = cv2.FONT_HERSHEY_SIMPLEX
    sss = ["x=", "y="]
    ppp = [x_in_image, y_in_image]
    for i in range(2):
        s = "{:.2f}".format(res_P_board[i,0])
        TEST_ROWS = 300+i*30
        TEST_COLS = 350
        img_annotated = cv2.putText(
                    img_annotated, sss[i]+s, (TEST_COLS, TEST_ROWS), font, 
                    1, (255, 255, 255), 2, cv2.LINE_AA)

    cv2.imshow("object show in red dot", img_annotated)
    cv2.waitKey()
    cv2.destroyAllWindows()

if __name__=="__main__":
    main()
