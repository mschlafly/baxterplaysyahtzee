
import numpy as np
import numpy
import sys
import cv2
import numpy
from scipy.ndimage import label
from matplotlib import pyplot as plt
from ourlib_cv2 import find_object_in_middle

import os, sys
CURRENT_PATH=os.path.join( os.path.dirname(__file__) )+"/"
if CURRENT_PATH[0]=="/":
    CURRENT_PATH="."+CURRENT_PATH
# sys.path.append(PACKAGE_PATH)
from ourlib_cv2 import *

if __name__=="__main__":
    
    filename = CURRENT_PATH+"/imgmid3.png"
    print filename
    img = cv2.imread(filename)
    # img= equalize_image(img)
    # plt.imshow(img),plt.colorbar(),plt.show()

    # detect object
    mask, rect = find_object_in_middle(img, ratio_RADIUS_TO_CHECK=3, disextend=50)
    

    # mean_val = cv2.mean(img,mask = mask)
    # print "mean_val", mean_val
    
    def create_blob_detector(blob_min_area=12, 
                         blob_min_int=0.0, blob_max_int=1.0, blob_th_step=5):
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.minArea = blob_min_area
        params.maxArea = 30
        params.filterByCircularity = False
        params.filterByColor = False
        params.filterByConvexity = False
        params.filterByInertia = False
        # blob detection only works with "uint8" images.
        params.minThreshold = int(blob_min_int*255)
        params.maxThreshold = int(blob_max_int*255)
        params.thresholdStep = blob_th_step
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3:
            return cv2.SimpleBlobDetector(params)
        else:
            return cv2.SimpleBlobDetector_create(params) 

    imgmask=img.copy()
    imgmask[mask==0]=0

    small = crop_image(imgmask, rect)
    # small = equalize_image(small, filter_size=2)
    detector = create_blob_detector()
    keypoints = detector.detect(small)

    print "number of dots", len(keypoints)
    
    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(small, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    # Show keypoints
    cv2.imshow("Keypoints", im_with_keypoints)
    cv2.waitKey(0)

    TRY_THRESHOLD=False
    if TRY_THRESHOLD:
        mean_color=get_color_median(img, mask, rect)    
        print mean_color

        minx=min(rect[:,0])
        maxx=max(rect[:,0])
        miny=min(rect[:,1])
        maxy=max(rect[:,1])

        for i in range(miny,maxy):
            for j in range(minx,maxx):
                if mask[i,j]==0:
                    img[i,j]=mean_color
                cb.append(img[i,j,0])
                cg.append(img[i,j,1])
                cr.append(img[i,j,2])

        small=img[miny:maxy, minx:maxx]

        img[mask==0]=0
        plt.imshow(mask),plt.colorbar(),plt.show()

    
    TEST_SEG=False
    if TEST_SEG:

        labeled_img=color_seg(small,  
            neighbor = 8,
            sigma = 0.5,
            K = 100.0,
            min_size = 10)
        colored_img=rander_color(labeled_img)
        plt.imshow(colored_img),plt.colorbar(),plt.show()


    # img=cv2.adaptiveThreshold(cv2.cvtColor(img,cv2.COLOR_RGB2GRAY))
    # ret,thresh1 = cv.threshold(img,127,255,cv.THRESH_BINARY)

    # color = ('b','g','r')
    # for i,col in enumerate(color):
    #     histr = cv2.calcHist([img],[i],None,[256-100],[100,256])
    #     plt.plot(histr,color = col)
    #     plt.xlim([0,256])
    # plt.show()

    # if res_rect is None:
    #     print "not find anything"
    # else:
    #     (center_x, center_y, radius_x, radius_y, angle)  = extract_rect(res_rect)
    #     # --------------
    #     image_for_display_object=cv2.drawContours(img, [res_rect], 0, [0,0,1], 2)
    #     plt.imshow(image_for_display_object),plt.colorbar(),plt.show()
