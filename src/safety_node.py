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
import parameter as parm

class Setpoint:
 
    def __init__(self, pub, rospy):
        self.pub = pub
        self.rospy = rospy
 
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
 
        try:
            thread.start_new_thread( self.navigate, () )
        except:
            print("Error: Unable to start thread")
 
        # TODO(simon): Clean this up.
        self.done = False
        self.done_evt = threading.Event()
        sub = rospy.Subscriber('/mavros/mocap/pose', PoseStamped, self.reached)
 
    def navigate(self):
        rate = self.rospy.Rate(10) # 10hz
 
        msg = PoseStamped()
        msg.header = Header() 
        msg.header.frame_id = "games_on_tracks"
        msg.header.stamp = rospy.Time.now()
 
        while 1:
            msg.pose.position.x = self.x
            msg.pose.position.y = self.y
            msg.pose.position.z = self.z
 
            # For demo purposes we will lock yaw/heading to north.
            yaw_degrees = 0  # North
            yaw = radians(yaw_degrees)
            quaternion = quaternion_from_euler(0, 0, yaw)
            msg.pose.orientation = Quaternion(*quaternion)
 
            self.pub.publish(msg)
 
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
 
 
    def reached(self, topic):
            #print topic.pose.position.z, self.z, abs(topic.pose.position.z - self.z)
            if abs(topic.pose.position.x - self.x) < 0.5 and abs(topic.pose.position.y - self.y) < 0.5 and abs(topic.pose.position.z - self.z) < 0.5:
                self.done = True
 
            self.done_evt.set()

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




def odrone_interface():
    rospy.Subscriber('/vicon_data', PoseStamped, safety_area)
    rospy.init_node('odrone_interface', anonymous=False)

    
    while True:
        key = raw_input("\n[GCS] ODRONE >> ")
        
        if key == 'a' :
            arm(True)

        elif key == 'd' :
            arm(False)

        elif key == 'w' :
            start_lqr(True)

        elif key == 's' :
            start_lqr(False)

        elif key == 'q' :
            start_lqr(False)
            arm(False)
            break


if __name__ == '__main__':
    try:
        odrone_interface()
    except rospy.ROSInterruptException:
        pass


