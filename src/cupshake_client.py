#!/usr/bin/env python

import sys
import rospy
from baxterplaysyahtzee.srv import *

def pour_the_cup_client():
    rospy.wait_for_service('pour_the_cup')
    try:
        pour_the_cup = rospy.ServiceProxy('pour_the_cup', CupShake)
        success = pour_the_cup()
        # return success
        print(success)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    print "Requesting cup pour..."

    pour_the_cup_client()

    print "Done!"
