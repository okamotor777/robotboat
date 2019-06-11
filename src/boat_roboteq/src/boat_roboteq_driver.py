#!/usr/bin/python
import serial
import rospy
import time
import os
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistWithCovarianceStamped
from geometry_msgs.msg import Vector3

# value used for publishing the light blinking message
lastSwitchVal = 0

# rcmode global variable
RCmode = 2

# begin the connection to the roboteq controller
try:
    ser = serial.Serial(
        port='/dev/serial/by-path/pci-0000:00:14.0-usb-0:2:1.0',
        baudrate=115200, #8N1
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    )
except:
    raise IOError

# reset the connection if need be
if (ser.isOpen()):
    ser.close()
ser.open()

# function moveCallback
# takes the /cmd_vel data as input
# returns nothing
def moveCallback(data):
    if (abs(data.linear.x) > 0.001 or abs(data.angular.z) > 0.001):
        speed = data.linear.x *1000 #linear.x is value between -1 and 1 and input to wheels is between -1000 and 1000
        if speed > 1000:
            speed = 1000
        elif speed < -1000:
            speed = -1000
                                            #1000 would give full speed range, but setting to lower value to better control robot
        turn = (data.angular.z + 0.009)*700*-1
        if turn > 1000:
            turn = 1000
        elif turn < -1000:
            turn = -1000

        cmd = '!G 1 ' + str(-speed) + '\r'
        ser.write(cmd)
        cmd = '!G 2 ' + str(-turn) + '\r'
        ser.write(cmd)

if __name__ == "__main__":
    # start the roboteq node
    rospy.init_node('igvc_roboteq', anonymous=True)

    # start the cmd_vel subscriber
    rospy.Subscriber("roboteq_driver/cmd", Twist, moveCallback)
    try:
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
          
            rate.sleep()
    except KeyboardInterrupt:
        ser.close()
        raise











			
