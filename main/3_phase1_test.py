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

#initialize ultrasonic class and begin polling sensors
usonic = usonic(0x70, 1, sim_mask = [False, True, True, True, True, True], sim_readings=[-1,100,600,600,600,600])
usonic.start()
time.sleep(2)
print usonic.readings

mavros.set_namespace()

#stereo = Stereo_Vision(cam_L_src=0,cam_R_src=1)

#initailize ROS publisher for setpoints at 10Hz
rospy.init_node('setpoint_position_demo')
mavros.set_namespace()  # initialize mavros module with default namespace
rate = rospy.Rate(10)
setpoint = Setpoint(sonar_readings=usonic.readings, jMAVSim = True, sen_threshold=100)

raw_input("Press any key to begin")

print("Altitude up")
setpoint.altitude_change(2, wait=True)
setpoint.pitch(50, wait=False, check_obs=True, store_final = True)

while setpoint.done == False and setpoint.obs_detected == False:
    pass

if setpoint.obs_detected == True:
    print "Phase 1 complete"
else:
    print "Drone crashed"

print "Mission complete"
