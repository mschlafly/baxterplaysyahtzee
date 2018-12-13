#!/usr/bin/env python

import sys
import rospy

def pour_the_cup_client():
    rospy.wait_for_service('pour_the_cup')
    try:
        pour_the_cup = rospy.ServiceProxy('pour_the_cup', pour_cup)
        working_or_not = pour_the_cup()
        return working_or_not
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    print "Requesting cup pour..."

    pour_the_cup_client()

    print "Done!"
