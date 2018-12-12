#!/usr/bin/env python2


import rospy
from baxterplaysyahtzee.srv import *

# a template for calling service
def call_service(service_name, service_type, args=None):
    rospy.wait_for_service(service_name)
    try:
        func = rospy.ServiceProxy(service_name, service_type)
        func(*args) if args else func() # call this service
    except rospy.ServiceException, e:
        print "Failed to call service:", service_name
        sys.exit()

if __name__=="__main__":
    rospy.init_node('nodeCV_example')

    SERVICE_NAME="/mycvCalibChessboardPose"

    while not rospy.is_shutdown():
        rospy.sleep(1.0)
        print "calling service: " + SERVICE_NAME
        call_service(SERVICE_NAME, CalibChessboardPose)


