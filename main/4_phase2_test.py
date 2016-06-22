import numpy
import cv2
import sys
import time
import rospy
import thread
import threading
import mavros
import numpy as np
import imutils

from geometry_msgs.msg import PoseStamped, Quaternion
from math import *
from std_msgs.msg import Header
from std_msgs.msg import String
from mavros.utils import *
from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler
from threading import Thread, Event


#from Stereo_Vision import Stereo_Vision
from MAVROS_navigation import Setpoint
from Stereo_Vision import Stereo_Vision
from Usonic import usonic
#stereo = Stereo_Vision(cam_L_src=0,cam_R_src=1)
usonic = usonic(0x70, 1, sim_mask = [False, True, True, True, True, True], sim_readings=[-1,100,600,600,600,600])

mavros.set_namespace()

#initialize sonar sensor network and vision system
#stereo = Stereo_Vision(cam_L_src=0,cam_R_src=1)
#usonic = usonic(stuff)

#initailize ROS publisher for setpoints at 10Hz
rospy.init_node('setpoint_position_demo')
mavros.set_namespace()  # initialize mavros module with default namespace
rate = rospy.Rate(10)
setpoint = Setpoint(sonar_readings=usonic.readings, jMAVSim = True)

raw_input("Press any key to begin")

print("Altitude up")
setpoint.altitude_change(2)
setpoint.pitch(5, wait=False, check_obs=True, store_final = True)

while setpoint.done == False and setpoint.obs_detected == False:
    pass

if setpoint.obs_detected == True:
    if usonic.readings[3] > 500:
        print "Rolling left"
        setpoint.roll(-2,check_obs=False, wait=True) # roll left 2m
        if usonic.readings[0] > 500:
            print "Front sensor clear, reading is %s" %usonic.readings[0]
            setpoint.pitch(2, check_obs=False, wait=True)
        else:
            print "Front sensor not clear, hovering, regain manual control and return to home"
    elif usonic.readings[1] > 500:
        print "Rolling right"
        setpoint.roll(2,check_obs=False, wait=True) # roll right 2m
        if usonic.readings[0] > 500:
            print "Front sensor clear, reading is %s" % usonic.readings[0]
            setpoint.pitch(2, check_obs=False, wait=True)
        else:
            print "Front sensor not clear, hovering, regain manual control and return to home"
    elif usonic.readings[4] > 500:
        print "Ascending"
        setpoint.altitude_change(3, wait=True)     #increase altittude 2m
        if usonic.readings[0] > 500:
            print "Front sensor clear, reading is %s" % usonic.readings[0]
            setpoint.pitch(2, check_obs=False, wait=True)
        else:
            print "Front sensor not clear, hovering, regain manual control and return to home"
    else:
        print "No clear path, regain manual control"


else:
    print "No obstacle detected along path"

print "Mission complete"
