#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from ourlib_cv import myTrackbar

if __name__=="__main__":
    window_name='image'
    trackbar = myTrackbar(window_name)
    
    while(1):
        img=cv2.imread('test_cv/image2.png')
        
        LB, UB=trackbar.get_trackbar_values_LB_UB()

        mask=cv2.inRange(img,LB,UB)
        mask = cv2.cvtColor(mask,cv2.COLOR_GRAY2RGB)
        
        cv2.imshow(window_name,np.hstack([img,mask]))
        
        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"):
            break
    cv2.destroyAllWindows()
