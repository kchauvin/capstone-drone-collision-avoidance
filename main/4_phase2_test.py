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

front_threshold = 100
front_threshold_delta = 0

#test to left
#usonic = usonic(0x70, 5, disable_mask = [False, True, True, True, True, True], sim_readings=[-1,100,600,600,600,600], threshold=[front_threshold,500,500,500,500])
#test to right
usonic = usonic(0x70, 5, disable_mask = [False, True, True, True, True, True], sim_readings=[-1,600,600,50,600,600], threshold=[front_threshold,500,500,500,500])

mavros.set_namespace()

#initialize sonar sensor network and vision system
#stereo = Stereo_Vision(cam_L_src=0,cam_R_src=1)
usonic.start()

#initailize ROS publisher for setpoints at 10Hz
rospy.init_node('setpoint_position_demo')
mavros.set_namespace()  # initialize mavros module with default namespace
rate = rospy.Rate(10)
setpoint = Setpoint(sonar_obs_present=usonic.obs_present, jMAVSim = False)

trapped = False

def nav_decision(obs_present):
    #object to the right
    if obs_present[1]:
        print "Rolling left"
        setpoint.roll(2, check_obs=False, wait=True)  # roll left 2m
        usonic.threshold[0] = front_threshold + front_threshold_delta  # set front sensor threshold to 5M
        time.sleep(0.3)
        if not (obs_present[0]):
            print "Front sensor clear, reading is %s" % usonic.readings[0]
            print "Pitching forward"
        else:
            print "Front sensor not clear rolling left again"
            setpoint.roll(-2, check_obs=False, wait=True, delay=0.5)  # roll left 2m
            if obs_present[0]:
                print "Front sensor not clear reading is %s, regain manual control" % usonic.readings[0]
                trapped = True

    # object to the left
    elif obs_present[3]:
        print "Rolling right"
        setpoint.roll(-2, check_obs=False, wait=True)  # roll right 2m
        usonic.threshold[0] = front_threshold + front_threshold_delta  # set front sensor threshold to 5M
        time.sleep(0.3)
        if not (obs_present[0]):
            print "Front sensor clear, reading is %s" % usonic.readings[0]
            print "Pitching forward"
        else:
            print "Front sensor not clear rolling right again"
            setpoint.roll(-2, check_obs=False, wait=True, delay=0.5)  # roll right 2m
            if obs_present[0]:
                print "Front sensor not clear reading is %s, regain manual control" % usonic.readings[0]
                trapped = True

    # ascend
    elif not (obs_present[4]):
        print "Ascending"
        setpoint.altitude_change(3, wait=True, delay=0.5)  # increase altittude 3m
        usonic.threshold[0] = front_threshold + front_threshold_delta  # set front sensor threshold to 5M
        time.sleep(0.3)
        if not (obs_present[0]):
            print "Front sensor clear, reading is %s" % usonic.readings[0]
            print "Pitching forward"
        else:
            print "Front sensor not clear reading is %s, regain manual control" % usonic.readings[0]
            trapped = True
    else:
        print "No clear path, regain manual control"
        trapped = True

    usonic.threshold[0] = front_threshold # set front sensor threshold to 5M


set_home = raw_input("Press 'h' to set home 2M above current location")

if set_home == 'h':
    setpoint.set_home(alt=2)

raw_input("Press any key to begin")

#pitch forward 10M while checking for obstacles
print usonic.readings
setpoint.pitch(10, wait=False, check_obs=True, store_final = True)

while setpoint.done == False and setpoint.obs_detected == False:
    pass

if setpoint.obs_detected == True:
    setpoint.obs_detected = False
    time.sleep(2)

    #decide best navigation from sensor/camera readings
    nav_decision(usonic.obs_present)

    if not(trapped):
        #if drone didn't encounter trapped condition, pitch forward 4m to clear obstacle
        setpoint.pitch(4, wait=False, check_obs=True, store_final=False)

        while setpoint.done == False and setpoint.obs_detected == False:
            pass

        #while pitching past obstacle, drone encountered second obstacle
        if setpoint.obs_detected == True:
            setpoint.obs_detected = False
            # disable_mask = [False, False, False, True, True, True]
            time.sleep(1)

            # decide best navigation path from sensor/camera readings
            nav_decision(usonic.obs_present)

            if not(trapped):
                # usonic.disable_mask = [False, True, True, True, True, True]
                setpoint.pitch(4, wait=False, check_obs=True, store_final=False)
                while setpoint.done == False and setpoint.obs_detected == False:
                    pass

                if setpoint.obs_detected:
                    setpoint.obs_detected = False
                    print "On second attempt past obstacle, drone encountered obstacle, halting, regain manual control"
                    trapped = True

    # drone has maneuvered past obstacle and not trapped
    if not(trapped):
        setpoint.yaw_towards_dest(setpoint.x_final_dest, setpoint.y_final_dest)
        time.sleep(2)

        #navigate to final destination
        setpoint.set(setpoint.x_final_dest,setpoint.y_final_dest,setpoint.curr_z,setpoint.delta_yaw)

else:
    print "No obstacle detected along path"

print "Final destination reached"

#yaw toward start point
print "Returning to home"
print setpoint.x_home, setpoint.y_home

setpoint.yaw_towards_dest(setpoint.x_home, setpoint.y_home)
time.sleep(2)

setpoint.pitch(10, wait=False, check_obs=True, store_final=True)

if setpoint.obs_detected == True:
    setpoint.obs_detected = False
    # disable_mask = [False, False, False, True, True, True]
    time.sleep(2)

    # decide best navigation from sensor/camera readings
    nav_decision(usonic.obs_present)

    if not (trapped):
        # if drone didn't encounter trapped condition, pitch forward 4m to clear obstacle
        # usonic.disable_mask = [False, True, True, True, True, True]
        setpoint.pitch(4, wait=False, check_obs=True, store_final=False)

        while setpoint.done == False and setpoint.obs_detected == False:
            pass

        # while pitching past obstacle, drone encountered second obstacle
        if setpoint.obs_detected == True:
            setpoint.obs_detected = False
            time.sleep(2)

            # decide best navigation path from sensor/camera readings
            nav_decision(usonic.obs_present)

            if not (trapped):
                # usonic.disable_mask = [False, True, True, True, True, True]
                setpoint.pitch(4, wait=False, check_obs=True, store_final=False)

                while setpoint.done == False and setpoint.obs_detected == False:
                    pass

                if setpoint.obs_detected:
                    setpoint.obs_detected = False
                    print "On second attempt past obstacle, drone encountered obstacle, halting, regain manual control"
                    trapped = True




setpoint.yaw_abs(0)

print "Mission complete"
