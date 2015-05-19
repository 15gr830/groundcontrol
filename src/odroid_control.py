#!/usr/bin/env python
from __future__ import print_function

import thread, threading, time, struct, os, sys

import roslib; roslib.load_manifest('groundcontrol')
import rospy
import mavros
import argparse
import parameter as parm
from mavros.utils import *
from mavros import command
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from sensor_msgs.msg import Joy
from mavros.msg import State


class Quad_state:
    arm = False
    mode = 1

class Modes:
    grounded = 1
    takeoff = 2
    hovering = 3
    landing = 4
    intransit = 5

class Setpoint:

    def __init__(self, args, pub, rospy):
        self.pub = pub
        self.args = args
        self.rospy = rospy

        self.init_pose = [0,0,0]
        self.setpoint = self.init_pose
        self.setpoint_queue = []
        self.quad_state = Quad_state()
        self.mode = Modes()

        try:
            thread.start_new_thread( self.tx_sp, () )
            
            if self.args == 'keyboard':
                thread.start_new_thread( self.keyboard, () )
        except:
            print("Error: Unable to start thread")

        self.initialised = False
        self.done_event = threading.Event()
        self.rospy.Subscriber('/mavros/mocap/pose', PoseStamped, self.goal, queue_size=1)
        self.rospy.Subscriber('/vicon_data', PoseStamped, self.safety_area, queue_size=1)
        self.rospy.Subscriber('mavros/state', State, self.state, queue_size=1)
        
        if self.args == 'joystik':
            self.rospy.Subscriber('/joy', Joy, self.joystik, queue_size=1)


    def tx_sp(self):
        rate = self.rospy.Rate(10)

        sp = PoseStamped()
        sp.header = Header()
        sp.header.frame_id = "global_frame"
        sp.header.stamp = self.rospy.Time.now()

        while True:
            sp.pose.position.x = self.setpoint[0]
            sp.pose.position.y = self.setpoint[1]
            sp.pose.position.z = self.setpoint[2]

            self.pub.publish(sp)

            rate.sleep()


    def set(self, setpoint):

        for i in range(0,len(setpoint)):
            self.setpoint_queue.append(setpoint[i])


    def goal(self, topic):

        if not self.initialised :
            self.init_pose = [topic.pose.position.x, topic.pose.position.y, topic.pose.position.z]
            self.setpoint = self.init_pose
            self.quad_state.mode = self.mode.grounded

        elif len(self.setpoint_queue):
            for i in range(len(self.setpoint_queue[0])):
                self.setpoint[i] = self.setpoint_queue[0][i] + self.init_pose[i]

            if abs(topic.pose.position.x - self.setpoint[0]) < parm.threshold and abs(topic.pose.position.y - self.setpoint[1]) < parm.threshold and abs(topic.pose.position.z - self.setpoint[2]) < parm.threshold:
                self.setpoint_queue.pop(0)
        else:
            self.quad_state.mode = self.mode.hovering
            
        self.done_event.set()


    def arm(self,state):
        self.setpoint_queue = []
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

        if (abs(x) > parm.sandbox[0]) or (abs(y) > parm.sandbox[1]) or (z > parm.sandbox[2]) :
            while self.quad_state.arm:
                self.start_lqr(False)
                self.arm(False)

            rospy.loginfo("\n[GCS] QUAD OUTSIDE SANDBOX")

    def state(self,topic):
        self.quad_state.arm = topic.armed


    def joystik(self, topic):

        if topic.buttons[15] and not self.quad_state.arm :
            print("[QGC] ARMING")
            self.arm(True)
            self.start_lqr(True)

        elif topic.buttons[13] and self.quad_state.arm :
            print("[QGC] DISARMING")
            self.arm(False)
            self.start_lqr(False)

        elif topic.buttons[12] :
            if self.quad_state.mode is not self.mode.takeoff:
                print("[QGC] TAKEOFF")
                self.set(parm.takeoff)
                self.quad_state.mode = self.mode.takeoff

        elif topic.buttons[14] :
            if self.quad_state.mode is not self.mode.landing:
                print("[QGC] LANDING")
                self.setpoint_queue = []
                self.set(parm.landing)
                self.quad_state.mode = self.mode.landing

        elif topic.buttons[6] :
            if self.quad_state.mode is not self.mode.intransit:
                print("[QGC] Flying in squares")
                self.set(parm.square)
                self.quad_state.mode = self.mode.intransit

    def keyboard(self):
        print("\n<----CONTROL INPUTS---->")
        print("\n\t'a' = ARM")
        print("\t'd' = DISAM")
        print("\t't' = TAKEOFF")
        print("\t'l' = LAND")
        print("\t's' = Flying in square")
        print("\t'q' = QUIT")

        while True:
            try:
                key = raw_input("\n[GCS] ODRONE >> ")
                
                if key == 'a' :
                    print("[QGC] ARMING")
                    self.arm(True)
                    self.start_lqr(True)

                elif key == 'd' :
                    print("[QGC] DISARMING")
                    self.arm(False)
                    self.start_lqr(False)

                elif key == 't' :
                    print("[QGC] TAKEOFF")
                    self.set(parm.takeoff)

                elif key == 'l' :
                    print("[QGC] LANDING")
                    self.setpoint_queue = []
                    self.set(parm.landing)

                elif key == 'q' :
                    print("[QGC] QUITING")
                    self.arm(False)
                    self.start_lqr(False)
                    break
            except KeyboardInterrupt, SystemExit:
                print("\EXITING")


def main():
    pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
    rospy.init_node('odrone_interface', anonymous=False)

    Setpoint(sys.argv[1],pub,rospy)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


