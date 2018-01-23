#!/usr/bin/env python

import socket
import time
import rospy
from sensor_msgs.msg import JointState
import math as m

# HOST = '192.168.1.12'     # The remote host
# PORT = 30002              # The same port as used by the server

# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s.connect((HOST, PORT))

time.sleep(0.05)


# --------example joint angle command--------
# s.send ("movej([1.499063295, -0.05468042885,1.686686189, -0.04956735076,-4.76247993,3.127797186], a=1.0, v=0.1)" + "\n")
# --------example move_to_point command--------
# s.send ("movej(p[0.00, -0.32, -0.6, 2.22, -2.22, 0.00], a=0.2, v=0.1)" + "\n")


def move_to_waypoint(wp):
# sends the waypoint to the socket w/ a=0.2 and v=0.1
    wp_r = [i * m.pi/180 for i in wp.position] # way point (joint angles) in radians
    # s.send("movej([" + str(wp_r[0]) + ", " + str(wp_r[1]) + ", " + str(wp_r[2]) + ", " + str(wp_r[3]) + ", " + str(wp_r[4]) + ", " + str(wp_r[5]) + "], a=0.2, v=0.1)\n")
    print(" the command send to socket is: movej([" + str(wp_r[0]) + ", " + str(wp_r[1]) + ", " + str(wp_r[2]) + ", " + str(wp_r[3]) + ", " + str(wp_r[4]) + ", " + str(wp_r[5]) + "], a=0.2, v=0.1)\n")
    time.sleep(15)

   
def ur5_joint_angles_node():
# subscribes to ur5_joint_angles
    rospy.init_node('ur5_joint_angles_node', anonymous=True)
    rospy.Subscriber("ur5_joint_angles", JointState, move_to_waypoint)
    rospy.loginfo('Subscribing to ur5_joint_angles to move the ur5 arm')
    rospy.spin()

if __name__ == '__main__':
    try:
        ur5_joint_angles_node()
    except rospy.ROSInterruptException:
        pass
    # s.close()