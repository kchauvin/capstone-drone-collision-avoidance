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

from matplotlib import pyplot as plt
from geometry_msgs.msg import PoseStamped, Quaternion
from math import *
from std_msgs.msg import Header
from std_msgs.msg import String
from mavros.utils import *
from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler

from Stereo_Vision import Stereo_Vision
from MAVROS_navigation import Setpoint
#from Usonic import usonic

mavros.set_namespace()


#initialize sonar sensor network and vision system
stereo = Stereo_Vision(cam_L_src=0,cam_R_src=1)
#usonic = usonic(stuff)

#initailize ROS publisher for setpoints at 10Hz
pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
rospy.init_node('pose', anonymous=True)
rate = rospy.Rate(10)
setpoint = Setpoint(pub, rospy, sonar_readings=usonic.readings)

raw_input("Press any key to begin")


setpoint.altitude_change(2)


setpoint.pitch(5, wait=False, check_obs=True, sonar_readings=usonic.readings, store_final_dst = True)

while not(setpoint.done or setpoint.obs_detected):
    if setpoint.obs_detected == True:   #sonar detected object along path and halted motion
        if not(stereo.object_left_column):
            if usonic.readings[3] < 400:
                setpoint.roll(-2)
                print "Rolling right"
            elif usonic.readings[1] < 400:
                setpoint.roll(2)
                print "Rolling left"
            elif usonic.readings[5] < 400:
                setpoint.altitude_change(2)
                "Increasing altitude 2m, left and right paths blocked"
            elif usonic.readings[2] < 400:
                setpoint.pitch(-2)
                print "Pitching backwards, obstacle above and on left and right of drone"
            else
                print "Unable to determine path, loitering, regain manual control and land"

        elif stereo.object_right_column:

        elif stereo.object_top_row:

        elif stereo.object_mid_row:

        elif stereo.object_bottom_row:






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


