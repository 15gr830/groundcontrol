#!/usr/bin/env python
from __future__ import print_function

import thread, threading, time, struct, os, sys

import roslib; roslib.load_manifest('groundcontrol')
import rospy
import mavros
import argparse
import parameter as parm
from mavros.utils import *
from mavros import command, setpoint as sp
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from sensor_msgs.msg import Joy
from mavros.msg import State
from mavros.srv import CommandBool


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
        self.args = args
        self.pub = pub
        self.rospy = rospy

        self.setpoint = [0,0,0]
        self.setpoint_queue = []
        self.quad_state = Quad_state()
        self.mode = Modes()
        self.current_pose = PoseStamped()
        self.initialised = False

        try:
            thread.start_new_thread( self.send_setpoint, () )
            thread.start_new_thread( self.watchdog, () )
            
            if self.args == 'keyboard':
                thread.start_new_thread( self.keyboard, () )
        except:
            print("Error: Unable to start thread")

        self.done_event = threading.Event()
        self.rospy.Subscriber('/mavros/mocap/pose', PoseStamped, self.goal, queue_size=10)
        self.rospy.Subscriber('/vicon_data', PoseStamped, self.safety_area, queue_size=10)
        self.rospy.Subscriber('mavros/state', State, self.state, queue_size=10)
        
        if self.args == 'joystik':
            self.rospy.Subscriber('/joy', Joy, self.joystik, queue_size=10)


    def send_setpoint(self):
        rate = self.rospy.Rate(2)

        sp = PoseStamped()
        sp.header = Header()
        sp.header.frame_id = "global_frame"
        sp.header.stamp = self.rospy.Time.now()

        while True:
            # for i in range(0,3):
            #     if (abs(self.setpoint[i]) > parm.safezone[i]):
            #         self.setpoint[i] = copysign(parm.safezone[i], self.setpoint[i])
            
            sp.pose.position.x = self.setpoint[0]
            sp.pose.position.y = self.setpoint[1]
            sp.pose.position.z = self.setpoint[2]

            self.pub.publish(sp)

            rate.sleep()


    def set(self, setpoint):

        for i in range(0,len(setpoint)):
            self.setpoint_queue.append(setpoint[i])


    def goal(self, topic):
        self.initialised = True

        self.current_pose.header = topic.header
        self.current_pose.pose = topic.pose
        
        if not self.quad_state.arm:
            self.init_pose = [self.current_pose.pose.position.x, self.current_pose.pose.position.y, self.current_pose.pose.position.z]
            
            self.setpoint[0] = self.current_pose.pose.position.x
            self.setpoint[1] = self.current_pose.pose.position.y
            self.setpoint[2] = self.current_pose.pose.position.z

            self.quad_state.mode = self.mode.grounded

        elif len(self.setpoint_queue):
            for i in range(0,len(self.setpoint_queue[0])):
                self.setpoint[i] = self.setpoint_queue[0][i] + self.init_pose[i]

            if abs(topic.pose.position.x - self.setpoint[0]) < parm.threshold and abs(topic.pose.position.y - self.setpoint[1]) < parm.threshold and abs(topic.pose.position.z - self.setpoint[2]) < parm.threshold:
                self.setpoint_queue.pop(0)
        else:
            self.quad_state.mode = self.mode.hovering
            
        self.done_event.set()


    def arm(self, state):
        self.setpoint_queue = []
        try:
            arming_cl = self.rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
            ret = arming_cl(value=state)
        except rospy.ServiceException as ex:
            fault(ex)

        if not ret.success:
            self.rospy.loginfo("Request failed.")
        else:
            self.rospy.loginfo("Request success.")


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
            while self.quad_state.arm :
                self.start_lqr(False)
                self.arm(False)
                rospy.sleep(1)

            rospy.loginfo("\n[GCS] QUAD OUTSIDE SANDBOX")

    def state(self,topic):
        self.quad_state.arm = topic.armed

    def watchdog(self):
        rate = self.rospy.Rate(2)

        while True:
            if self.initialised:
                # print("watchdog running")
                tic = self.current_pose.header.seq 
                rate.sleep()
                toc = self.current_pose.header.seq

                if tic is toc and self.quad_state.arm:
                    print("[QGC] LOST GOT DATA")
                    self.initialised =False
                    while self.quad_state.arm:
                        self.arm(False)
                        self.start_lqr(False)
                        rospy.sleep(1)


    def joystik(self, topic):

        if topic.buttons[15] and not self.quad_state.arm and self.initialised :
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
                
                if key == 'a' and not self.quad_state.arm and self.initialised:
                    print("[QGC] ARMING")
                    self.arm(True)
                    self.start_lqr(True)

                elif key == 'd' and self.quad_state.arm :
                    print("[QGC] DISARMING")
                    self.arm(False)
                    self.start_lqr(False)

                elif key == 't' is not self.mode.takeoff :
                    print("[QGC] TAKEOFF")
                    self.set(parm.takeoff)
                    self.quad_state.mode = self.mode.takeoff

                elif key == 'l' and not self.mode.landing :
                    print("[QGC] LANDING")
                    self.setpoint_queue = []
                    self.set(parm.landing)
                    self.quad_state.mode = self.mode.landing

                elif key == 's' and not self.mode.intransit :
                    print("[QGC] Flying in squares")
                    self.set(parm.square)
                    self.quad_state.mode = self.mode.intransit

                elif key == 'q' :
                    print("[QGC] QUITING")
                    self.arm(False)
                    self.start_lqr(False)
                    break
            except KeyboardInterrupt, SystemExit:
                print("\EXITING")


def main():
    pub = sp.get_pub_position_local(queue_size=10, latch=True)
    rospy.init_node('odrone_interface', anonymous=False)

    Setpoint(sys.argv[1],pub,rospy)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


