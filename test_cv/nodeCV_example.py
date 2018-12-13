#!/usr/bin/env python2


# rosservice call /baxterplaysyahtzee/teleport_absolute 5.5400 5.5400 0.4636

# rosservice call /clear

# rosservice call /mycvCalibChessboardPose
# rosservice call /mycvGetObjectInImage




import rospy
from baxterplaysyahtzee.srv import *
from baxterplaysyahtzee.msg import *

# a template for calling service
def call_service(service_name, service_type, args=None):
    rospy.wait_for_service(service_name)
    try:
        func = rospy.ServiceProxy(service_name, service_type)
        func(*args) if args else func() # call this service
    except rospy.ServiceException, e:
        print "Failed to call service:", service_name
        print "The error is: ", e
        sys.exit()

if __name__=="__main__":
    rospy.init_node('nodeCV_example')

    while not rospy.is_shutdown():

        # ----------- Test service of Chessboard
        SERVICE_NAME="/mycvCalibChessboardPose"
        print "calling service: " + SERVICE_NAME
        call_service(SERVICE_NAME, CalibChessboardPose)

        # ----------- Get Object In Image
        # SERVICE_NAME="/mycvGetObjectInImage"
        # print "calling service: " + SERVICE_NAME
        # call_service(SERVICE_NAME, GetObjectInImage)

        # ----------- Get All Objects In Image
        # SERVICE_NAME="/mycvGetAllObjectsInImage"
        # print "calling service: " + SERVICE_NAME
        # call_service(SERVICE_NAME, GetAllObjectsInImage)


        # ----------- Test service of Chessboard
        SERVICE_NAME="/mycvGetObjectInBaxter"
        print "calling service: " + SERVICE_NAME
        call_service(SERVICE_NAME, GetObjectInBaxter)

        print "Test round over, sleep for 10 seconds."
        rospy.sleep(10)
