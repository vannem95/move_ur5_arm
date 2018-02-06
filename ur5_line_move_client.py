#!/usr/bin/env python
import sys
import rospy
from move_ur5_arm.srv import *

def ur5_line_move_client(mode, distance):
    rospy.wait_for_service('ur5_line_move_service')
    try:
        add_two_ints = rospy.ServiceProxy('ur5_line_move_service', ur5_line_move)
        resp1 = add_two_ints(mode, distance)
        return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [mode distance] {distance in meters}"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        mode = sys.argv[1]
        distance = float(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Moving to %s or %s from last point"%(mode,distance)
    print "%s %s,%s"%(ur5_line_move_client(mode, distance),mode,distance)
