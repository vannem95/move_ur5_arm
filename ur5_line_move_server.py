#!/usr/bin/env python

import socket
import time
import rospy
from geometry_msgs.msg import Twist
from move_ur5_arm.srv import *
import math as m

last_point=[0,0,0,0,0,0]

# HOST = '192.168.1.12'     # The remote host
# PORT = 30002              # The same port as used by the server

# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s.connect((HOST, PORT))

# time.sleep(0.05)

# --------example joint angle command--------
# s.send ("movej([1.499063295, -0.05468042885,1.686686189, -0.04956735076,-4.76247993,3.127797186], a=1.0, v=0.1)" + "\n")
# --------example move_to_point command--------
# s.send ("movej(p[0.00, -0.32, -0.6, 2.22, -2.22, 0.00], a=0.2, v=0.1)" + "\n")

def move_to_waypoint(req):
# sends the waypoint to the socket w/ a=0.2 and v=0.1
    global last_point
    if "home"==req.a:
        point = [1,2,3,4,5,6]
    elif "midpoint"==req.a:
        point = [1,2,3,4,5,6]
    elif "destination"==req.a:
        point = [1,2,3,4,5,6]
    elif "up"==req.a:
        point = [1,2,3+req.b,4,5,6]
    elif "down"==req.a:
        point = [1,2,3-req.b,4,5,6]
    elif "left"==req.a:
        point = [1,2+req.b,3,4,5,6]
    elif "right"==req.a:
        point = [1,2-req.b,3,4,5,6]
    elif "forward"==req.a:
        point = [1+req.b,2,3,4,5,6]
    elif "backward"==req.a:
        point = [1-req.b,2,3,4,5,6]
    else

    last_point = point

    # s.send("movel(p[" + str(point[0]) + ", " + str(point[1]) + ", " + str(point[2]) + ", " + str(point[3]) + ", " + str(point[4]) + ", " + str(point[5]) + "], a=0.2, v=0.1)\n")
    print(" the command send to socket is: movel(p[" + str(point[0]) + ", " + str(point[1]) + ", " + str(point[2]) + ", " + str(point[3]) + ", " + str(point[4]) + ", " + str(point[5]) + "], a=0.2, v=0.1)\n")
    # time.sleep(15)


def ur5_line_move_server():
# starts a service that takes direction(up/down/left/right/forward/backward)/checkpoint() and distance/0 
    rospy.init_node('ur5_line_move_server', anonymous=True)
    serv = rospy.Service('ur5_line_move', ur5_line_move, move_to_waypoint)
    rospy.loginfo('Service initiated...\nArguments are \n1. Direction/Checkpoint {{home/midpoint/destination} {up/down/left/right/forward/backward -- from destination} }\n2. Distance(in m) {'0' zero if a checkpoint is chosen}')
    rospy.spin()

if __name__ == '__main__':
    try:
        ur5_line_move_server()
    except rospy.ROSInterruptException:
        pass
    s.close()