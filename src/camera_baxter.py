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

class BaxterCamera:
    def __init__(self):
        print("Initialing class BaxterCamera")
        self.bridge = CvBridge()

        # params
        self.cnt=0 #count images

        # selet camera

        image_topic = "/cameras/"+camera+"/image"

        # set a subscriber
        self.sub = rospy.Subscriber(image_topic, Image, self.image_callback)

    def image_callback(self, msg):

        self.cnt+=1

        # Convert your ROS Image message to OpenCV2
        I = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        print("reading the {}th image".format(self.cnt))

        # image processing
        # row,col,_ = I.shape
        # hsv_I = cv2.cvtColor(I, cv2.COLOR_BGR2HSV)
    
        # show image
        cv2.imshow('I',I)
        cv2.waitKey(1)

        if self.cnt%10==0:
            cv2.imwrite('src/me495_group3/images/baxter_image_'+str(self.cnt)+'.jpg',I)


def main():

	rospy.init_node('open_camera')
	baxter_camera = BaxterCamera()
 	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
