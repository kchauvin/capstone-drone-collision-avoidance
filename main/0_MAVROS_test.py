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
import os

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
usonic = usonic(0x70, 1, disable_mask = [False, True, True, True, True, True], sim_readings=[-1,100,600,600,600,600])
usonic.start()
time.sleep(2)
print usonic.readings

mavros.set_namespace()

#stereo = Stereo_Vision(cam_L_src=0,cam_R_src=1)

#initailize ROS publisher for setpoints at 10Hz
rospy.init_node('setpoint_position_demo')
mavros.set_namespace()  # initialize mavros module with default namespace
rate = rospy.Rate(10)
setpoint = Setpoint(jMAVSim = True)

while 1:
	print "Curr X: %s" %setpoint.curr_x
	print "Curr Y: %s" %setpoint.curr_y
	print "Curr Z: %s" %setpoint.curr_z
	os.system('clear')
