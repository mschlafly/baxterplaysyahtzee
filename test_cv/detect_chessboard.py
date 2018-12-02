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
OUTPUT_FOLDER="images_results/"
TEST_MULTIPLE_IMAGES=False

def get_all_image_filenames(idx_folder):
    str_folders = [
        "images_baxter_head",
        "images_baxter_left_hand",
        "test_cv",
    ]
    str_folder = str_folders[idx_folder]
    return glob.glob("./"+str_folder+"/*.jpg")

def get_single_filename(image_foldername, image_filename):
    return [PROJECT_PATH+image_foldername+"/"+image_filename,]

def set_images_to_read():
    if TEST_MULTIPLE_IMAGES==True: # input all images under a folder
        images=get_all_image_filenames(idx_folder=1)
    else:
        image_filename='baxter_left_hand_camera_image.jpg'
        image_foldername = "test_cv"
        images=get_single_filename(image_foldername, image_filename)
    return images

def main():

    # set image/images to read
    images=set_images_to_read()

    # Init chessboard locator
    cl=ChessboardLocator(STR_CAMERA_TYPE="Left",SQUARE_SIZE=0.015)

    # Iterate through all images
    for cnt_img, IMAGE_FILENAME in enumerate(images):

        # print
        print "\n{}/{}".format(cnt_img,len(images))

        # read image
        img = cv2.imread(IMAGE_FILENAME)

        # detect chessboard
        flag, R, p, img_annotated = cl.locate_chessboard(
            img,
            OUTPUT_FOLDER = OUTPUT_FOLDER,
            OUTPUT_IMAGE_FILENAME=IMAGE_FILENAME
        )

if __name__=="__main__":
    main()
