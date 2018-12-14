#!/usr/bin/env python

# Save baxter's camera's video into images at frequency about 2 image/second.
# Choose a camera first.

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from matplotlib import pyplot as plt
from std_msgs.msg import String

# import os, sys
# PACKAGE_PATH=os.path.join( os.path.dirname(__file__) )+"/../"
# sys.path.append(PACKAGE_PATH)

cameras = ["left_hand_camera", "head_camera"]
CAMERA = cameras[0]
CAMERA_TOPIC = "/cameras/"+CAMERA+"/image"

# CAMERA_TOPIC="/usb_cam/image_raw"

SKIP = 20

class BaxterCamera:
    def __init__(self):
        print("Initialing class BaxterCamera")
        self.bridge = CvBridge()

        # params
        self.cnt=0 #count images

        # set a subscriber
        self.sub = rospy.Subscriber(CAMERA_TOPIC, Image, self.image_callback)

    def image_callback(self, msg):

        self.cnt += 1

        # Convert your ROS Image message to OpenCV2
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        rospy.loginfo("reading the {}th image".format(self.cnt))

        # image processing
        # row,col,_ = img.shape
        # hsv_img= cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
        # show image
        cv2.imshow('img',img)
        cv2.waitKey(1)

        if self.cnt % SKIP== 0:
            s="{:04d}".format(self.cnt)
            filename=PACKAGE_PATH+"/images/"+CAMERA+'_a'+s+'.jpg'
            res=cv2.imwrite(filename, img)
            print("---------- save image to %s ------------"%filename)
            None

def main():
	rospy.init_node('open_camera')
	print "init node"
	BaxterCamera()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main()
