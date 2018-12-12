
import cv2

image=cv2.imread("lib_image_seg/cup1.png")
small = cv2.resize(image, (0,0), fx=0.5, fy=0.5) 
cv2.imwrite("lib_image_seg/cup0.png", small)


