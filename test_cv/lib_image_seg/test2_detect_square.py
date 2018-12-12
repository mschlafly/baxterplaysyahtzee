
import numpy as np
import numpy
import sys
import cv2
import numpy
from scipy.ndimage import label
from matplotlib import pyplot as plt
from ourlib_cv2 import rander_color, find_squares, find_square, refine_image_mask, ellipse2xyangle

import os, sys
CURRENT_PATH=os.path.join( os.path.dirname(__file__) )
# sys.path.append(PACKAGE_PATH)

def angle_to_180(theta):
    if theta>90:
        theta-=180
    elif theta<-90:
        theta+=180
    return theta


if __name__=="__main__":
    
    filename = CURRENT_PATH+"/image1.png"
    img = cv2.imread(filename)
    height, width, depth = img.shape

    labeled_img=cv2.imread(CURRENT_PATH+"/labeled_image1_.png", 0)
    colored_img=rander_color(labeled_img)
    rows, cols=labeled_img.shape

    # find the squares
    res_ellipse, res_rects, res_xs, res_ys, res_angles = find_squares(labeled_img)
    n_rects=len(res_rects)

    DRAW_SQURAE_TO_IMAGE=True
    for i in range(n_rects):

        rect=res_rects[i]
        angle=res_ellipse[i][2]

        xy=res_ellipse[i][0]
        x=xy[0]
        y=xy[1]

        color=[0,0,255]
        if DRAW_SQURAE_TO_IMAGE:
            cv2.drawContours(colored_img, [rect], 0, color, 2)
        print "x={%.2f}, y={%.2f}, angle={%.2f}\n"%(x, y, angle)
    
    if DRAW_SQURAE_TO_IMAGE:
        plt.imshow(colored_img),plt.colorbar(),plt.show()

    # # refine image mask using grab cut
    # mask_res=refine_image_mask(img, rect)

    # # find the square again
    # ellipse, rect = find_square(mask_res,  
    #     MIN_AREA=100,
    #     ERROR_HOW_SQUARE=0.8, # len/width
    #     ERROR_TOTAL_AREA=0.8, # total_area/(len*width)
    #     ERROR_TOTAL_AREA2=0.8, # total_area/(the logical "and" area of square and segmentation result)
    # )
    # if ellipse is None:
    #     print "not find anything"
    #     plt.imshow(mask_res),plt.colorbar(),plt.show()
    # else:
    #     x,y,angle = ellipse2xyangle(ellipse)

    #     # add rect plot again
    #     cv2.drawContours(img, [rect], 0, color, 2)

    #     # final plot image
    #     plt.imshow(img),plt.colorbar(),plt.show()
    

    # cv2.imshow(filename, img)
    # cv2.waitKey()
    # cv2.destroyAllWindows()
