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


def command(data):
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

    elif key.buttons[11] :
        # Takeoff
        setpoint.set(0.0, 0.0, parm.takeoff_alt)

    elif key.buttons[10] :
        # Land
        setpoint.set(0.0, 0.0, parm.takeoff_alt/2)
        setpoint.set(0.0, 0.0, 0.0)




class Setpoint:

    def __init__(self, pub, rospy):
        self.pub = pub
        self.rospy = rospy

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.threshold = 200.0

        try:
            thread.start_new_thread( self.tx_sp, () )
        except:
            print("Error: Unable to start thread")

        self.done = False
        self.done_event = threading.Event()
        rospy.Subscriber('/mavros/mocap/pose', PoseStamped, self.goal)

    def tx_sp(self):
        rate = self.rospy.Rate(10)

        sp = PoseStamped()
        sp.header = Header()
        sp.header.frame_id = "global_frame"
        sp.header.stamp = rospy.Time.now()

        while True:
            sp.pose.position.x = self.x
            sp.pose.position.y = self.y
            sp.pose.position.z = self.z

            self.pub.publish(sp)

            rate.sleep()

    def set(self, x, y, z, delay=0, wait=True):
        self.done = False
        self.x = x
        self.y = y
        self.z = z

        if wait:
            rate = rospy.Rate(5)
            while not self.done:
                rate.sleep()
        
        time.sleep(delay)

    def goal(self, topic):
        if abs(topic.pose.position.x - self.x) < self.threshold and abs(topic.pose.position.y - self.y) < self.threshold and abs(topic.pose.position.z - self.z) < self.threshold:
            self.done = True
            
        self.done_event.set()


def odrone_interface():
    rospy.Subscriber('/vicon_data', PoseStamped, safety_area)
    rospy.Subscriber('/joy', Joy, command)
    
    pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

    rospy.init_node('odrone_interface', anonymous=False)

    global setpoint
    setpoint = Setpoint(pub,rospy)

    # setpoint.set(0.0, 0.0, 1.0)
    # setpoint.set(0.0, 1.0, 0.0)
    test()

    rospy.spin()

if __name__ == '__main__':
    try:
        odrone_interface()
    except rospy.ROSInterruptException:
        pass


