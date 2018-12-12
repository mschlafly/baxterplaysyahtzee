#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from PIL import Image
import cv2
import numpy as np
from lib_main import *

from ourlib_cv2 import *

import os, sys
CURRENT_PATH=os.path.join( os.path.dirname(__file__) )+"/"
# sys.path.append(PACKAGE_PATH)

if __name__ == '__main__':
    Filename="image1"
    # Filename="cup0"
    
    s = '_'.join(sys.argv[1:])
    print s
    FILENAME_OUT=Filename+"_"+s+".png"

    print "reading image from", CURRENT_PATH+Filename+".png"
    img=cv2.imread(CURRENT_PATH+Filename+".png")
    img=equalize_image(img)
    # img = cv2.resize(img, (0,0), fx=0.5, fy=0.5) 

    # segment image
    labeled_img=color_seg(img)
    print "write out the image:", CURRENT_PATH+"/labeled_"+FILENAME_OUT
    cv2.imwrite(CURRENT_PATH+"/labeled_"+FILENAME_OUT,labeled_img)

    colored_image = rander_color(labeled_img)

    # output
    out=np.hstack([img, colored_image])

    print "write out the image:", CURRENT_PATH+FILENAME_OUT
    cv2.imwrite(CURRENT_PATH+FILENAME_OUT,out)
    cv2.imshow(FILENAME_OUT, out)
    cv2.waitKey()
    cv2.destroyAllWindows()

