#!/usr/bin/env python
from __future__ import print_function

import thread, threading, time, struct

import rospy
import mavros
import parameter as parm
from mavros.utils import *
from mavros import command
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from sensor_msgs.msg import Joy


class Setpoint:

    def __init__(self, pub, rospy):
        self.pub = pub
        self.rospy = rospy

        self.init_pose = [0,0,0]
        self.setpoint = self.init_pose
        self.setpoint_queue = []

        try:
            thread.start_new_thread( self.tx_sp, () )
        except:
            print("Error: Unable to start thread")

        self.initialised = False
        self.done_event = threading.Event()
        self.rospy.Subscriber('/mavros/mocap/pose', PoseStamped, self.goal)
        self.rospy.Subscriber('/vicon_data', PoseStamped, self.safety_area)
        self.rospy.Subscriber('/joy', Joy, self.joystik)


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

        elif len(self.setpoint_queue):
            for i in range(len(self.setpoint_queue[0])):
                self.setpoint[i] = self.setpoint_queue[0][i] + self.init_pose[i]

            if abs(topic.pose.position.x - self.setpoint[0]) < parm.threshold and abs(topic.pose.position.y - self.setpoint[1]) < parm.threshold and abs(topic.pose.position.z - self.setpoint[2]) < parm.threshold:
                self.setpoint_queue.pop(0)
            
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
                    self.start_lqr(False)
                    self.arm(False)

                    rospy.loginfo("\n[GCS] QUAD OUTSIDE SANDBOX")
                    rospy.sleep(2)


    def joystik(self, topic):

        if topic.buttons[15] :
            print("[QGC] ARMING")
            self.arm(True)
            self.start_lqr(True)

        elif topic.buttons[13] :
            print("[QGC] DISARMING")
            self.arm(False)
            self.start_lqr(False)

        elif topic.buttons[11] :
            print("[QGC] TAKEOFF")
            self.set(parm.takeoff)

        elif topic.buttons[10] :
            print("[QGC] LANDING")
            self.setpoint_queue = []
            self.set(parm.landing)

        elif topic.buttons[6] :
            print("[QGC] Flying in squares")
            self.set(parm.square)


def main():
    pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
    rospy.init_node('odrone_interface', anonymous=False)

    Setpoint(pub,rospy)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


