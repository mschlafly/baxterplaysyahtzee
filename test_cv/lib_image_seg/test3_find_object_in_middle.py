
import numpy as np
import numpy
import sys
import cv2
import numpy
from scipy.ndimage import label
from matplotlib import pyplot as plt
from ourlib_cv2 import find_object_in_middle

import os, sys
CURRENT_PATH=os.path.join( os.path.dirname(__file__) )
# sys.path.append(PACKAGE_PATH)
from ourlib_cv2 import *

if __name__=="__main__":
    
    filename = CURRENT_PATH+"/imgmid2.png"
    img = cv2.imread(filename)
    # img= equalize_image(img)
    # plt.imshow(img),plt.colorbar(),plt.show()

    # detect object
    mask, res_rect = find_object_in_middle(img, ratio_RADIUS_TO_CHECK=3, disextend=50)

    if res_rect is None:
        print "not find anything"
    else:
        (center_x, center_y, radius_x, radius_y, angle)  = extract_rect(res_rect)
        # --------------
        image_for_display_object=cv2.drawContours(img, [res_rect], 0, [0,0,1], 2)
        plt.imshow(image_for_display_object),plt.colorbar(),plt.show()
