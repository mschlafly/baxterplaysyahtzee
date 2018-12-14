
#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import cv2

from lib_image_seg.lib_main import *
from lib_image_seg.lib_cv_detection import *

import os, sys
CURRENT_PATH=os.path.join( os.path.dirname(__file__) )+"/"
if CURRENT_PATH[0]=='/':
    CURRENT_PATH="."+CURRENT_PATH

if __name__ == '__main__':

    Filename=CURRENT_PATH+"/lib_image_seg/image1"
    
    s = ""
    # s = '_'.join(sys.argv[1:])
    FILENAME_OUT=Filename+"_"+s+".png"

    img=cv2.imread(Filename+".png")

    rects, labeled_img = find_all_objects(img, FLAG_DRAW_SQURAE_TO_IMAGE=False, IMAGE_RESIZE_SCALE=0.5)
    colored_image = find_all_objects_then_draw(rects, labeled_img, IF_PRINT=True)
    cv2.imshow("colored image", colored_image), cv2.waitKey(), cv2.destroyAllWindows()


    