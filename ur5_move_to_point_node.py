#!/usr/bin/env python

import socket
import time
import rospy
from geometry_msgs.msg import Twist
import math as m

HOST = '192.168.1.12'     # The remote host
PORT = 30002              # The same port as used by the server

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

time.sleep(0.05)


# --------example joint angle command--------
# s.send ("movej([1.499063295, -0.05468042885,1.686686189, -0.04956735076,-4.76247993,3.127797186], a=1.0, v=0.1)" + "\n")
# --------example move_to_point command--------
# s.send ("movej(p[0.00, -0.32, -0.6, 2.22, -2.22, 0.00], a=0.2, v=0.1)" + "\n")


def move_to_waypoint(wp):
# sends the waypoint to the socket w/ a=0.2 and v=0.1
    s.send("movej(p[" + str(wp.linear.x) + ", " + str(wp.linear.y) + ", " + str(wp.linear.z) + ", " + str(wp.angular.x) + ", " + str(wp.angular.y) + ", " + str(wp.angular.z) + "], a=0.2, v=0.1)\n")
    print(" the command send to socket is: movej(p[" + str(wp.linear.x) + ", " + str(wp.linear.y) + ", " + str(wp.linear.z) + ", " + str(wp.angular.x) + ", " + str(wp.angular.y) + ", " + str(wp.angular.z) + "], a=0.2, v=0.1)\n")
    time.sleep(15)

   
def ur5_move_to_point_node():
# subscribes to ur5_joint_angles
    rospy.init_node('ur5_move_to_point_node', anonymous=True)
    rospy.Subscriber("ur5_way_point", Twist, move_to_waypoint)
    rospy.loginfo('Subscribing to ur5_way_point(meters) to move the ur5 arm')
    rospy.spin()

if __name__ == '__main__':
    try:
        ur5_move_to_point_node()
    except rospy.ROSInterruptException:
        pass
    s.close()