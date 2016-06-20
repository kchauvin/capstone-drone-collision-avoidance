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
from mavros.utils import *
from mavros import setpoint as SP

class Setpoint:

    def __init__(self, sonar_readings=[]):

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.yaw = 0.0

        # publisher for mavros/setpoint_position/local
        self.pub = SP.get_pub_position_local(queue_size=10)
        # subscriber for mavros/local_position/local
        self.sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
                                    SP.PoseStamped, self.reached)

        try:
            thread.start_new_thread( self.navigate, () )
        except:
            print "Error: Unable to start thread"
                                    
                                    
        #self.sonar_readings = sonar_readings
        self.obs_detected = False
        
        #current coordinates
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_z = 0.0
        
        #coordinates for original setpoint, stored to continue to desired setpoint if interrupted
        self.x_final_dest = 0.0
        self.y_final_dest = 0.0
        self.z_final_dest = 0.0
        self.yaw_final_dest = 0.0

        self.done = False
        self.done_evt = threading.Event()

    def navigate(self):
        rate = rospy.Rate(10) # 10hz

        msg = SP.PoseStamped(
            header=SP.Header(
                frame_id="base_footprint",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()),    # stamp should update
        )
        #msg.header = Header()
        #msg.header.frame_id = "base_footprint"
        #msg.header.stamp = rospy.Time.now()

        while 1:
            #print self.curr_x, self.curr_y, self.curr_z
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

        #rate = rospy.Rate(5) #was not present in sample class

        if wait:
            print self.curr_x, self.curr_y, self.curr_z
            rate = rospy.Rate(5)
            while not self.done and not rospy.is_shutdown():
                rate.sleep()

        time.sleep(delay)

    def reached(self, topic):
        #self.curr_x = topic.pose.position.x
        #self.curr_y = topic.pose.position.y
        #self.curr_z = topic.pose.position.z
        #if abs(topic.pose.position.x - self.x) < 0.5 and abs(topic.pose.position.y - self.y) < 0.5 and abs(topic.pose.position.z - self.z) < 0.5:
        #    self.done = True
        #self.done_evt.set()
        def is_near(msg, x, y):
            rospy.logdebug("Position %s: local: %d, target: %d, abs diff: %d",
                           msg, x, y, abs(x - y))
            return abs(x - y) < 0.5
        
        self.curr_x = topic.pose.position.x
        self.curr_y = topic.pose.position.y
        self.curr_z = topic.pose.position.z 
            
        if is_near('X', topic.pose.position.x, self.x) and \
           is_near('Y', topic.pose.position.y, self.y) and \
           is_near('Z', topic.pose.position.z, self.z):
            self.done = True
            self.done_evt.set()


    #def _local_position_callback(self, topic):
    #    self.curr_x = topic.pose.position.x
    #    self.curr_y = topic.pose.position.y
    #    self.curr_z = topic.pose.position.z

    def set_rel(self, rel_x, rel_y, rel_z, rel_yaw, delay=0, wait=False):
        self.set(self.x+rel_x, self.y+rel_y, self.z+rel_z, self.yaw+rel_yaw, delay=delay, wait=wait)

    def pitch(self, step, delay=0, wait=True, check_obs=False, sonar_readings = [], store_final=False):
        new_x = self.x + step*cos(radians(self.yaw))
        new_y = self.y + step*sin(radians(self.yaw))

        if store_final == True:
            self.store_final_dest(new_x, new_y, self.z, self.yaw)

        if check_obs == True:
            self.set(new_x, new_y, self.z, self.yaw, delay, wait=False) #set new setpoint

            while(self.done == False):  #check for obstacles along path until dest
                if (step > 0):          #forward motion, check forward sonar sensor
                    if (sonar_readings[0] < 200):
                        self.halt()
                        print "Obstacle detected along path"
                        self.obs_detected = True
                else:                   #backwards motion, check forward sonar sensor
                    if (sonar_readings[2] < 200):
                        self.halt()
                        print "Obstacle detected along path"
                        self.obs_detected = True
        else:
            self.set(new_x, new_y, self.z, self.yaw, delay, wait=wait) #set new setpoint


    def roll(self, step, delay=0, wait=True, check_obs=False, sonar_readings = [], store_final=False):
        new_x = self.x + step*sin(radians(self.yaw))
        new_y = self.y + step*cos(-radians(self.yaw))


        if store_final == True:
            self.store_final_dest(new_x, new_y, self.z, self.yaw)

        if check_obs == True:
            self.set(new_x, new_y, self.z, self.yaw, delay, wait=False) #set new setpoint

            while(self.done == False):  #check for obstacles along path until dest
                if (step > 0):          #motion to the right, check right sonar sensor
                    if (sonar_readings[1] < 200):
                        self.halt()
                        print "Obstacle detected along path"
                        self.obs_detected = True
                else:                   #motion to the left, check left sonar sensor
                    if (sonar_readings[3] < 200):
                        self.halt()
                        print "Obstacle detected along path"
                        self.obs_detected = True
        else:
            self.set(new_x, new_y, self.z, self.yaw, delay, wait=False) #set new setpoint

        self.set(new_x, new_y, self.z, self.yaw, delay, wait=wait)

    def yaw_ch(self, step, delay=0, wait=True):
        new_yaw = self.yaw + step
        self.set(self.x, self.y, self.z, new_yaw, delay, wait=wait)

    def altitude_change(self,step, delay=0, wait=True, check_obs=False, sonar_readings = []):
        new_z = self.z + step

        if check_obs == True:
            self.set(self.x, self.y, self.z, self.yaw, delay, wait=False) #set new setpoint

            while(self.done == False):  #check for obstacles along path until dest
                if (step > 0):          #motion to the right, check right sonar sensor
                    if (sonar_readings[1] < 200):
                        self.halt()
                        print "Obstacle detected along path"
                        self.obs_detected = True
                else:                   #motion to the left, check left sonar sensor
                    if (sonar_readings[3] < 200):
                        self.halt()
                        print "Obstacle detected along path"
                        self.obs_detected = True
        else:
            self.set(self.x, self.y, new_z, self.yaw, delay, wait=False) #set new setpoint
        #takeoff and land can use altitude_change function, simulator likes to set 0 at 25m, may need to subtract 25 from desired height

    def home(self, alt=0, delay=0, wait=True):
        self.set(0.0, 0.0, alt, 0.0, 0, wait=wait)
        #returns home and hovers at 2 m in the air

    def halt(self, delay=0):
        #sub_localpos = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), SP.PoseStamped, self._local_position_callback)
        sub_localpos = rospy.Subscriber('/mavros/local_position/local', PoseStamped, self._local_position_callback)
        time.sleep(delay)
        print self.curr_x, self.curr_y, self.curr_z
        self.set(self.curr_x, self.curr_y, self.curr_z, self.yaw)

    def store_final_dest(self, x, y, z, yaw):
        self.x_final_dest = x
        self.y_final_dest = y
        self.z_final_dest = z
        self.yaw_final_dest = yaw



