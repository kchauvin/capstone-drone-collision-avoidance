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
from mavros.utils import *
from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler

mavros.set_namespace()

class Setpoint:

    def __init__(self, pub, rospy):
        self.pub = pub
        self.rospy = rospy

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_z = 0.0
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

    def set(self, x, y, z, yaw, delay=0, wait=False):
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
        self.curr_x = topic.pose.position.x
        self.curr_y = topic.pose.position.y
        self.curr_z = topic.pose.position.z
        if abs(topic.pose.position.x - self.x) < 0.5 and abs(topic.pose.position.y - self.y) < 0.5 and abs(topic.pose.position.z - self.z) < 0.5:
            self.done = True

        self.done_evt.set()


    def _local_position_callback(self, topic):
        self.curr_x = topic.pose.position.x
        self.curr_y = topic.pose.position.y
        self.curr_z = topic.pose.position.z

    def set_rel(self, rel_x, rel_y, rel_z, rel_yaw, delay=0, wait=False):
        setpoint.set(self.x+rel_x, self.y+rel_y, self.z+rel_z, self.yaw+rel_yaw, delay=delay, wait=wait)

    def pitch(self, step, delay=0, wait=True):
        new_x = self.x + step*cos(radians(self.yaw))
        new_y = self.y + step*sin(radians(self.yaw))
        setpoint.set(new_x, new_y, self.z, self.yaw, delay, wait=wait)

    def roll(self, step, delay=0, wait=True):
        new_x = self.x + step*sin(radians(self.yaw))
        new_y = self.y + step*cos(-radians(self.yaw))
        setpoint.set(new_x, new_y, self.z, self.yaw, delay, wait=wait)

    def yaw_ch(self, step, delay=0, wait=True):
        new_yaw = self.yaw + step
        setpoint.set(self.x, self.y, self.z, new_yaw, delay, wait=wait)

    def altitude_change(self,step, delay=0, wait=True):
        new_z = self.z + step
        setpoint.set(self.x, self.y, new_z, self.yaw, 0, wait=wait)
        #takeoff and land can use altitude_change function, simulator likes to set 0 at 25m, may need to subtract 25 from desired height

    def home(self, alt=0, delay=0, wait=True):
        setpoint.set(0.0, 0.0, alt, 0.0, 0, wait=wait)
        #returns home and hovers at 2 m in the air

    def halt(self, delay=0):
        self.sub_localpos = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
                                    SP.PoseStamped, self._local_position_callback)
        time.sleep(delay)
        print self.curr_x, self.curr_y, self.curr_z
        setpoint.set(self.curr_x, self.curr_y, self.curr_z, self.yaw)

#initailize ROS publisher for setpoints at 10Hz
pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
rospy.init_node('pose', anonymous=True)
rate = rospy.Rate(10)
setpoint = Setpoint(pub, rospy)

raw_input()
setpoint.altitude_change(2)
setpoint.pitch(5, wait=False)

range


while(setpoint.done == False and #sonarclear)
    if (#frontsonarnotclear):
        setpoint.halt()
            
    else
        pass

setpoint.yaw_ch(180)
setpoint.pitch(5, wait=False)
setpoint.yaw_ch(180)

)


# setpoint.pitch(100,0)
# time.sleep(1)
# setpoint.halt()

#print "Stopping Motion"
#setpoint.roll(0,0)


#setpoint.altitude_change(50)


