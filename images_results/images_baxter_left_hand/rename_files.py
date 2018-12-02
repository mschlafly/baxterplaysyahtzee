

#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import glob

output_name='cam_head'

images = glob.glob('./*.jpg')

cnt_img=0
for fname in images:
    cnt_img+=1
    img = cv2.imread(fname)
    cv2.imshow('img',img)
    cv2.imwrite('img'+str(cnt_img)+'.png',img)
  
