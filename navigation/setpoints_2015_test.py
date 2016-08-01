import numpy
import cv2
import sys
import time
import rospy
import thread
import threading
import mavros

from geometry_msgs.msg import PoseStamped, Quaternion
from math import *
from std_msgs.msg import Header
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler

class Setpoint:

    def __init__(self, pub, rospy):
        self.pub = pub
        self.rospy = rospy

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0

        try:
            thread.start_new_thread( self.navigate, () )
        except:
            print "Error: Unable to start thread"

        self.done = False
        self.done_evt = threading.Event()
        sub = rospy.Subscriber('/mavros/local_position/local', PoseStamped, self.reached)

    def navigate(self):
        rate = self.rospy.Rate(10) # 10hz

        msg = PoseStamped()
        msg.header = Header()
        msg.header.frame_id = "base_footprint"
        msg.header.stamp = rospy.Time.now()

        while 1:

            quaternion = quaternion_from_euler(0,0,radians(self.yaw))
            msg.pose.position.x = self.x
            msg.pose.position.y = self.y
            msg.pose.position.z = self.z
            msg.pose.orientation = Quaternion(*quaternion)

            self.pub.publish(msg)

            rate.sleep()

    def set(self, x, y, z, yaw, delay=0, wait=True):
        self.done = False
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw

        if wait:
            rate = rospy.Rate(5)
            while not self.done:
                rate.sleep()

        time.sleep(delay)

    def reached(self, topic):
        print topic.pose.position.z, self.z
        if abs(topic.pose.position.x - self.x) < 0.5 and abs(topic.pose.position.y - self.y) < 0.5 and abs(topic.pose.position.z - self.z) < 0.5:
            self.done = True

        self.done_evt.set()

    def pitch(self, step, delay):
        new_x = self.x + step*sin(radians(self.yaw))
        new_y = self.y + step*cos(radians(self.yaw))
        setpoint.set(new_x, new_y, self.z, self.yaw, delay)

    def roll(self, step, delay):
        new_x = self.x + step*cos(radians(self.yaw))
        new_y = self.y + step*sin(-radians(self.yaw))
        setpoint.set(new_x, new_y, self.z, self.yaw, delay)

    def yaw_ch(self, step, delay):
        new_yaw = self.yaw + step
        setpoint.set(self.x, self.y, self.z, new_yaw, delay)

    def altitude_change(self,step):
        new_z = self.z + step
        setpoint.set(self.x, self.y, new_z, self.yaw, 0)
        #takeoff and land can use altitude_change function, simulator likes to set 0 at 25m, may need to subtract 25 from desired height

    def home(self):
        setpoint.set(0.0, 0.0, 2.0,0.0, 0)
        #returns home and hovers at 5 m in the air

pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
rospy.init_node('pose', anonymous=True)
rate = rospy.Rate(10)
setpoint = Setpoint(pub, rospy)

#setpoint.pitch(50,0)
#print "done"
#setpoint.home()
#setpoint.roll(50,0)

#setpoint.altitude_change(50)
