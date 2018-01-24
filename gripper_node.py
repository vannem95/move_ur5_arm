#!/usr/bin/env python

import serial
import time
import rospy
from std_msgs.msg import Bool

device = rospy.get_param("~device")

ser = serial.Serial(port=device,baudrate=115200,timeout=1,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS)

time.sleep(0.05)

def open_close(switch):
# opens / closes the gripper
    if switch.data:
        print "Closing gripper"
        ser.write("\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29")
        time.sleep(1)

    else:
        print "Opening gripper"
        ser.write("\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19")
        time.sleep(1)
    

def gripper_node():
# subscribes to gripper_state topic
    rospy.init_node('gripper_node', anonymous=True)
    rospy.Subscriber("gripper_state", Bool, open_close)
    rospy.loginfo('Subscribing to gripper_state to open/close the gripper')
    rospy.spin()

if __name__ == '__main__':
    try:
        gripper_node()
    except rospy.ROSInterruptException:
        pass
