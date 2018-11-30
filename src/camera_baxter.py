#!/usr/bin/env python

# Save baxter's camera's video into images at frequency about 2 image/second.
# Choose a camera first.
if 0:
    camera="left_hand_camera"
else:
    camera="head_camera"

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from matplotlib import pyplot as plt
from std_msgs.msg import String

cameras = ["left_hand_camera", "head_camera"]

SKIP = 20
CAMERA = cameras[0]
CAMERA_TOPIC = "/cameras/"+CAMERA+"/image"

class BaxterCamera:
    def __init__(self):
        print("Initialing class BaxterCamera")
        self.bridge = CvBridge()

        # params
        self.cnt=0 #count images

        # select camera

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
            cv2.imwrite('images/baxter_image_'+str(self.cnt)+'.jpg', img)

def main():
	rospy.init_node('open_camera')
	baxter_camera = BaxterCamera()
 	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
