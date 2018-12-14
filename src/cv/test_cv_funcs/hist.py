import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread('test_cv/image3.jpg')
(r,c,d)=img.shape


TEST_HIST_EQUAL=True
TEST_MASK=False
TEST_PLOT_HIST=False
TEST_ORB=False
TEST_GRABCUT=True

if TEST_HIST_EQUAL:
    img_yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)

    # equalize the histogram of the Y channel
    img_yuv[:,:,0] = cv2.equalizeHist(img_yuv[:,:,0])

    # convert the YUV image back to RGB format
    img_output = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)
    img =img_output


    kernel = np.ones((5,5),np.float32)/25
    img = cv2.filter2D(img,-1,kernel)

    cv2.imshow("hist equalization", img)
    cv2.waitKey()

if TEST_MASK:
    mask = np.zeros(img.shape[:2], np.uint8)
    mask[100:r, 100:c] = 255
    masked_img = cv2.bitwise_and(img,img,mask = mask)
    cv2.imshow("masked_img",masked_img)
    cv2.waitKey(1000)
    img=masked_img

if TEST_PLOT_HIST:
    color = ('b','g','r')
    for i,col in enumerate(color):
        histr = cv2.calcHist([img],[i],None,[256],[0,256])
        plt.plot(histr,color = col)
        plt.xlim([0,256])
    plt.show()

if TEST_ORB:
    # Initiate STAR detector
    orb = cv2.ORB_create()

    # find the keypoints with ORB
    kp = orb.detect(img,None)

    # compute the descriptors with ORB
    kp, des = orb.compute(img, kp)

    # draw only keypoints location,not size and orientation
    img2 = cv2.drawKeypoints(img, kp, None, color=(0,255,0), flags=0)
    plt.imshow(img2)
    plt.show()
    
if TEST_GRABCUT:
    mask = np.zeros(img.shape[:2],np.uint8)

    bgdModel = np.zeros((1,65),np.float64)
    fgdModel = np.zeros((1,65),np.float64)

    rect = (100,300,200,100)
    cv2.grabCut(img,mask,rect,bgdModel,fgdModel,5,cv2.GC_INIT_WITH_RECT)

    mask2 = np.where((mask==2)|(mask==0),0,1).astype('uint8')
    img = img*mask2[:,:,np.newaxis]

    plt.imshow(img),plt.colorbar(),plt.show()
cv2.destroyAllWindows()