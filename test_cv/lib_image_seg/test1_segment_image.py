#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from PIL import Image
import cv2
import numpy as np
from lib_main import *

from ourlib_cv2 import *

if __name__ == '__main__':
    Filename="image30"
    # Filename="cup0"
    
    s = '_'.join(sys.argv[1:])
    print s
    FILENAME_OUT=Filename+"_"+s+".png"


    img=cv2.imread(Filename+".png")
    img=equalize_image(img)

    # segment image
    labeled_img=color_seg(img)
    cv2.imwrite("labeled_"+FILENAME_OUT,labeled_img)

    colored_image = rander_color(labeled_img)

    # output
    out=np.hstack([img, colored_image])

    cv2.imwrite(FILENAME_OUT,out)
    cv2.imshow(FILENAME_OUT, out)
    cv2.waitKey()
    cv2.destroyAllWindows()

