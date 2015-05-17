#!/usr/bin/env python
from __future__ import print_function

import rospy
import thread
import threading
import time
import mavros
import struct
import numpy as np
from mavros.utils import *
from mavros import command
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import parameter as parm


def arm(state):
    try:
        ret = command.arming(value=state)
    except rospy.ServiceException as ex:
        fault(ex)

    if not ret.success:
        fault("Request failed. Check mavros logs")

    return ret

def start_lqr(state):
    if state:
      start = 1
    else:
      start = -1

    try:
        ret = command.long(command=30002, confirmation=1,
                      param1=start,
                      param2=0,
                      param3=0,
                      param4=0,
                      param5=0,
                      param6=0)
    except rospy.ServiceException as ex:
        fault(ex)

def safety_area(data):
    x = data.pose.position.x
    y = data.pose.position.y
    z = data.pose.position.z

    abs_x = np.absolute(x)
    abs_y = np.absolute(y)

    if (abs_x > parm.sandbox[0]) or (abs_y > parm.sandbox[1]) or (z > parm.sandbox[2]) :
                start_lqr(False)
                arm(False)
                rospy.loginfo("\n[GCS] QUAD OUTSIDE SANDBOX")
                rospy.sleep(2)


def control(data):
    key = Joy()
    key.buttons = data.buttons

    if key.buttons[15] :
        arm(True)

    elif key.buttons[13] :
        arm(False)

    elif key.buttons[12] :
        start_lqr(True)

    elif key.buttons[14] :
        start_lqr(False)




def odrone_interface():
    rospy.Subscriber('/vicon_data', PoseStamped, safety_area)
    rospy.Subscriber('/joy', Joy, control)
    rospy.init_node('odrone_interface', anonymous=False)

    rospy.spin()

if __name__ == '__main__':
    try:
        odrone_interface()
    except rospy.ROSInterruptException:
        pass


