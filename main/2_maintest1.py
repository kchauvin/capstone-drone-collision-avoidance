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

#from matplotlib import pyplot as plt
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
#from Usonic import usonic

mavros.set_namespace()


#initialize sonar sensor network and vision system
#stereo = Stereo_Vision(cam_L_src=0,cam_R_src=1)
#usonic = usonic(stuff)

#initailize ROS publisher for setpoints at 10Hz
rospy.init_node('setpoint_position_demo')
mavros.set_namespace()  # initialize mavros module with default namespace
rate = rospy.Rate(10)
setpoint = Setpoint(sonar_readings=[500,500,500,500,500,500], jMAVSim = True)

raw_input("Press any key to begin")

setpoint.altitude_change(515)
print("Altitude up")
setpoint.pitch(100, wait=False, check_obs=False, sonar_readings=[500,500,500,500,500], store_final = True)

raw_input("Press any key to halt")
setpoint.halt()

print "im dumb"
#setpoint.yaw_ch(180)
#setpoint.pitch(5, wait=False)
#setpoint.yaw_ch(180)


# setpoint.pitch(100,0)
# time.sleep(1)
# setpoint.halt()

#print "Stopping Motion"
#setpoint.roll(0,0)


#setpoint.altitude_change(50)


