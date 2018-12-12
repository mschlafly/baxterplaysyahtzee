#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import cv2

from lib_image_seg.lib_main import *
from lib_image_seg.ourlib_cv2 import *

if __name__ == '__main__':

    Filename="lib_image_seg/image30"
    
    s = ""
    # s = '_'.join(sys.argv[1:])
    FILENAME_OUT=Filename+"_"+s+".png"

    img=cv2.imread(Filename+".png")
    height, width, depth = img.shape

    # pre processing
    img=equalize_image(img)

    # segment image -> labeled_image
    print "start segmenting image"
    labeled_img=color_seg(img,neighbor = 8,
        sigma = 0.5,
        K = 1000.0,
        min_size = 100)
    
    colored_image= rander_color(labeled_img)

    cv2.imshow("labeled image", colored_image)
    cv2.waitKey(10)

    # find the squares
    res_ellipse, res_rects, res_xs, res_ys, res_angles = find_squares(labeled_img)
    n_rects=len(res_rects)
    print "n_rects", n_rects

    # draw
    DRAW_SQURAE_TO_IMAGE=True
    if DRAW_SQURAE_TO_IMAGE:
        for i in range(n_rects):

            rect=res_rects[i]

            x=res_xs[i]
            y=res_ys[i]
            angle=res_angles[i]

            color=[0,0,255]
            cv2.drawContours(img, [rect], 0, color, 2)
            print "x={%.2f}, y={%.2f}, angle={%.2f}\n"%(x, y, angle)



    cv2.imshow("final seg", img)
    cv2.waitKey()
    cv2.destroyAllWindows()
