
#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from PIL import Image
import cv2
import numpy as np
from lib_main import *

from ourlib_cv2 import *

import os, sys
CURRENT_PATH=os.path.join( os.path.dirname(__file__) )+"./"
# sys.path.append(PACKAGE_PATH)

if __name__ == '__main__':
    Filename="imgmid3"
    # Filename="cup0"
    
    s = '_'.join(sys.argv[1:])
    print s
    FILENAME_OUT=Filename+"_"+s+".png"

    print "reading image from", CURRENT_PATH+Filename+".png"
    img=cv2.imread(CURRENT_PATH+Filename+".png")
    img=equalize_image(img)
    # img = cv2.resize(img, (0,0), fx=0.5, fy=0.5) 

    # segment image
    rects, labeled_img=find_all_objects(img)
    print "write out the image:", CURRENT_PATH+"/labeled_"+FILENAME_OUT
    cv2.imwrite(CURRENT_PATH+"/labeled_"+FILENAME_OUT,labeled_img)
    
    colored_image = find_all_objects_then_draw(rects, labeled_img, IF_PRINT=False)


    # ---------------
    rect=rects[1]
    (center_x, center_y, radius_x, radius_y, angle)  = extract_rect(rect)
    print "rect",rect
    print (center_x, center_y, radius_x, radius_y, angle) 
        
    # dots
    labeled_img = cv2.resize(labeled_img, (0,0), fx=2, fy=2)
    mask=labeled_img==labeled_img[int(center_y),int(center_x)]
    blank_image = np.zeros(mask.shape, np.uint8)
    blank_image[mask]=255
    mask=blank_image

    cv2.imshow(FILENAME_OUT, mask)
    cv2.waitKey()
    
    # mask=labeled_img==labeled_img[int(center_y/2),int(center_x/2)]
    # blank_image = np.zeros(mask.shape, np.uint8)
    # blank_image[mask]=255
    # blank_image = cv2.resize(blank_image, (0,0), fx=2, fy=2)
    # blank_image[blank_image<128]=0
    # blank_image[blank_image>=128]=1
    # mask=blank_imagegit 
    rect_int= rect.astype(np.uint32)

    print "rect_int",rect_int

    # ndots=detect_dots(img, mask, rect_int) # detect dots

    imgmask=img.copy()
    imgmask[mask==0]=0
    small = crop_image(imgmask, rect_int)
    # small = equalize_image(small, filter_size=2)
    detector = create_blob_detector()
    keypoints = detector.detect(small)

    cv2.imshow(FILENAME_OUT, small)
    cv2.waitKey()

    # color
    rgbcolor=get_color_median(img, mask, rect_int)
    
    print "ndots", ndots
    print "rgbcolor",rgbcolor

    out=colored_image

    print "write out the image:", CURRENT_PATH+FILENAME_OUT
    cv2.imwrite(CURRENT_PATH+FILENAME_OUT,out)
    cv2.imshow(FILENAME_OUT, out)
    cv2.waitKey()
    cv2.destroyAllWindows()

