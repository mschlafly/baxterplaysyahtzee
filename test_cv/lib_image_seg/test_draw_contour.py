

import cv2
import numpy as np


img = cv2.imread('star.png',0)
# img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)


ret,thresh = cv2.threshold(img,127,255,0)
im2, contours, hierarchy = cv2.findContours(thresh, 1, 2)

cnt=contours[0]
print "cnt=",cnt

M = cv2.moments(cnt)
print "Moments=",M

cx = int(M['m10']/M['m00'])
cy = int(M['m01']/M['m00'])

area = cv2.contourArea(cnt)

perimeter = cv2.arcLength(cnt,True)

epsilon = 0.1*cv2.arcLength(cnt,True)
approx = cv2.approxPolyDP(cnt,epsilon,True)

hull = cv2.convexHull(cnt)

x,y,w,h = cv2.boundingRect(cnt)
cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)


rect = cv2.minAreaRect(cnt)
box = cv2.boxPoints(rect)
box = np.int0(box)

print "rect=",rect
print "box=",box
# rect= ((208.0000457763672, 208.00001525878906), (4.242640495300293, 1.4142134189605713), -45.0)
# box= [[207 210]
#  [206 209]
#  [209 206]
#  [210 207]]

cv2.drawContours(img,[box],0,(0,0,255),2)


cv2.imshow("final",img)
cv2.waitKey()
cv2.destroyAllWindows()


