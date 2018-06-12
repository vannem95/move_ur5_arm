#!/usr/bin/env python

import socket
import time
import rospy
import serial
import math as m

from geometry_msgs.msg import Twist, PoseStamped
from move_ur5_arm.srv import *
from std_msgs.msg import Bool



#########################################################################################
#########################################################################################
############################### UR5 Socket Connection ###################################
#########################################################################################
#########################################################################################

HOST = '192.168.1.6'     # The remote host
PORT = 30002             # Zacobria has more info about the other ports and what they are good for.

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
time.sleep(0.05)

#########################################################################################



#########################################################################################
#########################################################################################
############################### Gripper Connection ######################################
#########################################################################################
#########################################################################################

ser = serial.Serial(port="/dev/ttyUSB0",baudrate=115200,timeout=1,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS)
time.sleep(0.05)

ser.write("\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30")
data_raw = ser.readline()
time.sleep(0.01)
ser.write("\x09\x03\x07\xD0\x00\x01\x85\xCF")
data_raw = ser.readline()
time.sleep(0.5)
print "Activating Gripper"
print "Now closing gripper.."
ser.write("\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29")
data_raw = ser.readline()
time.sleep(0.5)
print "Now opening gripper.."
ser.write("\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19")
data_raw = ser.readline()
time.sleep(0.5)
print "Gripper ready"

#########################################################################################



#########################################################################################
#########################################################################################
############################### Initializing points #####################################
#########################################################################################
#########################################################################################

last_point        = [-332.32/1000,-106.95/1000,-52.41/1000,0.0607,-2.7290,-0.0871]
home_point        = [-332.32/1000,-106.95/1000,-52.41/1000,0.0607,-2.7290,-0.0871]
grasp_point       = [-564.06/1000,-89.90/1000,123.39/1000,0.1956,4.7207,0.0671]

x_alligned        = 150.01/1000
y_alligned        = -52.00/1000
z_alligned        = 417.12/1000

correction_range  = 100/1000


x_correction      = 0.0
y_correction      = 0.0
z_correction      = 0.0

#########################################################################################


# --------example joint angle command--------
# s.send ("movej([1.499063295, -0.05468042885,1.686686189, -0.04956735076,-4.76247993,3.127797186], a=1.0, v=0.1)" + "\n")
# --------example move_to_point command--------
# s.send ("movej(p[0.00, -0.32, -0.6, 2.22, -2.22, 0.00], a=0.2, v=0.1)" + "\n")

def range_checker(value,range):

    if (value < -range) or (value > range):
        output = 0
    else:
        output = value
    return output


def marker_callback(point):
    global x_correction
    global y_correction
    global z_correction

    global x_alligned
    global y_alligned
    global z_alligned

    global correction_range


    x_marker = point.pose.position.x
    y_marker = point.pose.position.y
    z_marker = point.pose.position.z

    x_correction = x_marker - x_alligned
    y_correction = y_marker - y_alligned
    z_correction = z_marker - z_alligned


def move_to_waypoint(req):
# sends the waypoint to the socket w/ a=0.2 and v=0.2
    global last_point
    global home_point
    global grasp_point

    global x_correction
    global y_correction
    global z_correction

    if "home" == req.a:
        point = home_point

    elif "grasp" == req.a:
        point = grasp_point

    elif "allign" == req.a:
        point = grasp_point
        point[1] = point[1] + x_correction 
        point[2] = point[2] - y_correction 
        point[0] = point[0] - z_correction 

    elif "up" == req.a:
        last_point[2] = last_point[2] + (req.b/1000)
        point = last_point

    elif "down" == req.a:
        last_point[2] = last_point[2] - (req.b/1000)
        point = last_point

    elif "left" == req.a:
        last_point[1] = last_point[1] - (req.b/1000)
        point = last_point

    elif "right" == req.a:
        last_point[1] = last_point[1] + (req.b/1000)
        point = last_point

    elif "forward" == req.a:
        last_point[0] = last_point[0] - (req.b/1000)
        point = last_point

    elif "backward" == req.a:
        last_point[0] = last_point[0] + (req.b/1000)
        point = last_point

    elif "close" == req.a:
        ser.write("\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29")
        time.sleep(0.5)
        point = last_point
    elif "open" == req.a:
        ser.write("\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19")
        time.sleep(0.5)
        point = last_point
    else:
        print "wrong mode option. Please enter one of these for a: [modes: ""home/midpoint/destination/grasp/open/close"" directions: ""up/down/left/right/forward/backward""] and b: [ If you chose preset modes for a then ""0"". If you chose directions for a then ""distance(in mm)""] "

    last_point = point

    s.send("movel(p[" + str(point[0]) + ", " + str(point[1]) + ", " + str(point[2]) + ", " + str(point[3]) + ", " + str(point[4]) + ", " + str(point[5]) + "], a=0.2, v=0.2)\n")
    print(" the command send to socket is: movel(p[" + str(point[0]) + ", " + str(point[1]) + ", " + str(point[2]) + ", " + str(point[3]) + ", " + str(point[4]) + ", " + str(point[5]) + "], a=0.2, v=0.2)\n")
    time.sleep(2)
    return ur5_line_moveResponse("reached")


def ur5_line_move_server():
# starts a service that takes direction(up/down/left/right/forward/backward)/checkpoint() and distance/0 
    rospy.init_node('ur5_line_move_server', anonymous=True)
    serv = rospy.Service('ur5_line_move_service', ur5_line_move, move_to_waypoint)
    rospy.loginfo('Service initiated...\n\nArguments are \n\n1. Direction/mode {{ home / grasp / open / close} { up / down / left / right / forward / backward -- from last point} }\n\n2. Distance(in mm) {""0"" if a checkpoint is chosen}\n\n\n\n CHECK THE POSE OF THE ROBOT THEN CHOOSE THE MODE AND DISTANCE ACCORDINGLY')

    rospy.Subscriber('/visp_auto_tracker/object_position', PoseStamped, marker_callback)
    rospy.loginfo('Subscribing to Marker data for pickup allignment of the ur5 arm')
    rospy.spin()



if __name__ == '__main__':
    try:
        ur5_line_move_server()
    except rospy.ROSInterruptException:
        pass
    s.close()


