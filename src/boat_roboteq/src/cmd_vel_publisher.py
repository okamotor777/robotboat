!/usr/bin/env python
# -*- coding: utf-8 -*- 

# MAVLink joystick MIN MAX
TH_MAX = 1000.0
TH_MIN = 0.0
TH_CENTER = 500.0
YAW_MAX = 1000.0
YAW_MIN = -1000.0

LINEAR_X_MAX = 1.0 
ANGULAR_Z_MAX = 1.0

class boat_roboteq():
    def __init__(self):
        rospy.Subscriber('/mav/joystick', MAV_Joystick, self.ajk_joystick_subs, queue_size=1)

    def ajk_joystick_subs(self, msg):
        translation = (msg.throttle -TH_CENTER)/(TH_CENTER)*LINEAR_X_MAX
        steering = (-msg.yaw -YAW_MIN)/(YAW_MAX -YAW_MIN)*(AJK_MAX -AJK_MIN) + AJK_MIN
        self.joy_stamp = msg.stamp.secs +msg.stamp.nsecs/1000000000.0

if __name__ == '__main__':
    rospy.init_node('boat_roboteq')
    b = boat_roboteq()
    b.loop()
