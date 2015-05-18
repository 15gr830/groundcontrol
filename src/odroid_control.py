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


class Setpoint:

    def __init__(self, pub, rospy):
        self.pub = pub
        self.rospy = rospy

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.threshold = 0.2

        try:
            thread.start_new_thread( self.tx_sp, () )
        except:
            print("Error: Unable to start thread")

        self.done = False
        self.initialised = False
        self.done_event = threading.Event()
        self.rospy.Subscriber('/mavros/mocap/pose', PoseStamped, self.goal, queue_size=1)
        self.rospy.Subscriber('/vicon_data', PoseStamped, self.safety_area, queue_size=1)
        self.rospy.Subscriber('/joy', Joy, self.joystik)

    def tx_sp(self):
        rate = self.rospy.Rate(10)

        sp = PoseStamped()
        sp.header = Header()
        sp.header.frame_id = "global_frame"
        sp.header.stamp = self.rospy.Time.now()

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
            rate = self.rospy.Rate(3)
            while not self.done:
                rate.sleep()
        
        time.sleep(delay)

    def goal(self, topic):

        if not self.initialised :
            self.x = topic.pose.position.x
            self.y = topic.pose.position.y
            self.z = topic.pose.position.z

        elif abs(topic.pose.position.x - self.x) < self.threshold and abs(topic.pose.position.y - self.y) < self.threshold and abs(topic.pose.position.z - self.z) < self.threshold:
            self.done = True
            
        self.done_event.set()

    def arm(self,state):
        if state:
            self.initialised = True
        else:
            self.initialised = False

        try:
            ret = command.arming(value=state)
        except rospy.ServiceException as ex:
            fault(ex)

        if not ret.success:
            fault("Request failed. Check mavros logs")

        return ret

    def start_lqr(self, state):
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

    def safety_area(self,topic):
        x = topic.pose.position.x
        y = topic.pose.position.y
        z = topic.pose.position.z

        abs_x = np.absolute(x)
        abs_y = np.absolute(y)

        if (abs_x > parm.sandbox[0]) or (abs_y > parm.sandbox[1]) or (z > parm.sandbox[2]) :
                    self.start_lqr(False)
                    self.arm(False)
                    rospy.loginfo("\n[GCS] QUAD OUTSIDE SANDBOX")
                    rospy.sleep(2)


    def joystik(self, topic):
        key = Joy()
        key.buttons = topic.buttons

        if key.buttons[15] :
            print("[QGC] ARMING")
            self.arm(True)

        elif key.buttons[13] :
            print("[QGC] DISARMING")
            self.arm(False)

        elif key.buttons[12] :
            print("[QGC] STARTING LQR")
            self.start_lqr(True)

        elif key.buttons[14] :
            print("[QGC] STOPPING LQR")
            self.start_lqr(False)

        elif key.buttons[11] :
            # Takeoff
            print("[QGC] TAKEOFF")
            self.set(self.x, self.y, parm.takeoff_alt, wait=False)

        elif key.buttons[10] :
            # Land
            print("[QGC] LANDING")
            self.set(self.x, self.y, parm.landing_alt)
            self.set(self.x, self.y, 0.0)
            self.arm(False)

        elif key.buttons[9] :
            print("EMERGENCY SHUTDOWN")
            self.done = True
            self.arm(False)

        elif key.buttons[6] :
            print("[QGC] Flying in squares")

            for i in range(0,len(parm.square)) :
                self.set(self.x + parm.square[i][0], self.y + parm.square[i][1], self.z + parm.square[i][2])



def main():
    pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    rospy.init_node('odrone_interface', anonymous=False)

    setpoint = Setpoint(pub,rospy)
    # setpoint.arm(True)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


