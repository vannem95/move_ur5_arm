#!/usr/bin/env python

import socket
import time
import rospy
import serial
from geometry_msgs.msg import Twist
from move_ur5_arm.srv import *
import math as m
from std_msgs.msg import Bool


last_point = [110.38/1000,-323.24/1000,-620.19/1000,3.1299,-0.0030,-0.0158]
sleep_time = 0

HOST = '192.168.1.6'     # The remote host
PORT = 30002             # Zacobria has more info about the other ports and what they are good for.

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))


time.sleep(0.05)


ser = serial.Serial(port="/dev/ttyUSB0",baudrate=115200,timeout=1,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS)

time.sleep(0.05)

############# Auto Gripper test --- START --- #####################

ser.write("\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30")
data_raw = ser.readline()
time.sleep(0.01)
ser.write("\x09\x03\x07\xD0\x00\x01\x85\xCF")
data_raw = ser.readline()
time.sleep(1)
print "Activating Gripper"
print "Now closing gripper.."
ser.write("\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29")
data_raw = ser.readline()
time.sleep(2)
print "Now opening gripper.."
ser.write("\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19")
data_raw = ser.readline()
time.sleep(2)
print "Gripper ready"
############# Auto Gripper test --- END --- #####################

# --------example joint angle command--------
# s.send ("movej([1.499063295, -0.05468042885,1.686686189, -0.04956735076,-4.76247993,3.127797186], a=1.0, v=0.1)" + "\n")
# --------example move_to_point command--------
# s.send ("movej(p[0.00, -0.32, -0.6, 2.22, -2.22, 0.00], a=0.2, v=0.1)" + "\n")

def move_to_waypoint(req):
# sends the waypoint to the socket w/ a=0.2 and v=0.1
    global last_point
    global sleep_time
    if "home" == req.a:
        point = [110.38/1000,-363.24/1000,-610.19/1000,3.1299,-0.0030,-0.0158]
        sleep_time = 10
    elif "midpoint" == req.a:
        point = [110.39/1000,-363.23/1000,97.79/1000,3.1299,-0.0030,-0.0159]
        sleep_time = 7
    elif "destination" == req.a:
        point = [-324.98/1000,-127.38/1000,130.18/1000,2.2745,-2.1644,-0.0242]
        sleep_time = 7
    elif "grasp" == req.a:
        point = [-564.06/1000,-89.90/1000,123.39/1000,0.1956,4.7207,0.0671]
        sleep_time = 5
    elif "up" == req.a:
        last_point[2] = last_point[2] + (req.b/1000)
        point = last_point
        sleep_time = 5
    elif "down" == req.a:
        last_point[2] = last_point[2] - (req.b/1000)
        point = last_point
        sleep_time = 5
    elif "left" == req.a:
        last_point[1] = last_point[1] - (req.b/1000)
        point = last_point
        sleep_time = 5
    elif "right" == req.a:
        last_point[1] = last_point[1] + (req.b/1000)
        point = last_point
        sleep_time = 5
    elif "forward" == req.a:
        last_point[0] = last_point[0] - (req.b/1000)
        point = last_point
        sleep_time = 5
    elif "backward" == req.a:
        last_point[0] = last_point[0] + (req.b/1000)
        point = last_point
        sleep_time = 5
    elif "close" == req.a:
        ser.write("\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29")
        time.sleep(1)
        point = last_point
        sleep_time = 1
    elif "open" == req.a:
        ser.write("\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19")
        time.sleep(1)
        point = last_point
        sleep_time = 1
    else:
        print "wrong mode option. Please enter one of these for a: [modes: ""home/midpoint/destination/grasp/open/close"" directions: ""up/down/left/right/forward/backward""] and b: [ If you chose preset modes for a then ""0"". If you chose directions for a then ""distance(in mm)""] "

    last_point = point

    s.send("movel(p[" + str(point[0]) + ", " + str(point[1]) + ", " + str(point[2]) + ", " + str(point[3]) + ", " + str(point[4]) + ", " + str(point[5]) + "], a=0.2, v=0.1)\n")
    print(" the command send to socket is: movel(p[" + str(point[0]) + ", " + str(point[1]) + ", " + str(point[2]) + ", " + str(point[3]) + ", " + str(point[4]) + ", " + str(point[5]) + "], a=0.2, v=0.1)\n")
    time.sleep(sleep_time)
    return ur5_line_moveResponse("reached")


def ur5_line_move_server():
# starts a service that takes direction(up/down/left/right/forward/backward)/checkpoint() and distance/0 
    rospy.init_node('ur5_line_move_server', anonymous=True)
    serv = rospy.Service('ur5_line_move_service', ur5_line_move, move_to_waypoint)
    rospy.loginfo('Service initiated...\n\nArguments are \n\n1. Direction/mode {{home/midpoint/destination/grasp/open/close} {up/down/left/right/forward/backward -- from last point} }\n\n2. Distance(in mm) {""0"" if a checkpoint is chosen}\n\n\n\n CHECK THE POSE OF THE ROBOT THEN CHOOSE THE MODE AND DISTANCE ACCORDINGLY')
    rospy.spin()

if __name__ == '__main__':
    try:
        ur5_line_move_server()
    except rospy.ROSInterruptException:
        pass
    s.close()


