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

from MAVROS_navigation import Setpoint
from Stereo_Vision import Stereo_Vision
from Usonic import usonic

mavros.set_namespace()


front_threshold = 300
front_threshold_delta = 0

trapped = False
obs_count = 0

def nav_decision(obs_present, no_object_bottom_row, no_object_mid_row, no_object_top_row, no_object_left_column, no_object_mid_column, no_object_right_column):

    global obs_count
    global trapped
    stereo.save_frames(obs_count)
    obs_count += 1

    #object to the right, roll left
    if not(obs_present[3]) and (obs_present[1] or no_object_left_column):
        print "Rolling left"
        setpoint.roll(2, check_obs=False, wait=True, delay = 1)  # roll left 2m
        usonic.threshold[0] = front_threshold + front_threshold_delta  # set front sensor threshold to 5M
        time.sleep(2)
        if not (obs_present[0]):
            print "Front sensor clear, reading is %s" % usonic.readings[0]
            print "Pitching forward"
        else:
            print "Front sensor not clear rolling left again"
            print "reading is %s" % usonic.readings[0]
            setpoint.roll(2, check_obs=False, wait=True, delay=0.5)  # roll left 2m
            if obs_present[0]:
                print "Front sensor not clear reading is %s, regain manual control" % usonic.readings[0]
                trapped = True

    # object to the left, roll right
    elif not(obs_present[1]) and (obs_present[3] or no_object_right_column):
        print "Rolling right"
        setpoint.roll(-2, check_obs=False, wait=True, delay = 1)  # roll right 2m
        usonic.threshold[0] = front_threshold + front_threshold_delta  # set front sensor threshold to 5M
        time.sleep(2)
        if not (obs_present[0]):
            print "Front sensor clear, reading is %s" % usonic.readings[0]
            print "Pitching forward"
        else:
            print "Front sensor not clear rolling right again"
            print "reading is %s" % usonic.readings[0]
            setpoint.roll(-2, check_obs=False, wait=True, delay=0.5)  # roll right 2m
            if obs_present[0]:
                print "Front sensor not clear reading is %s, regain manual control" % usonic.readings[0]
                trapped = True

    # ascend if no object in top row of stereo
    elif no_object_top_row or (obs_present[1] and obs_present[3]):
        print "Ascending"
        setpoint.altitude_change(2, wait=True, delay=0.5)  # increase altittude 32
        usonic.threshold[0] = front_threshold + front_threshold_delta  # set front sensor threshold to 5M
        time.sleep(2)
        if not (obs_present[0]):
            print "Front sensor clear, reading is %s" % usonic.readings[0]
            print "Pitching forward"
        else:
            print "Front sensor not clear reading is %s, regain manual control" % usonic.readings[0]
            trapped = True

    #stereo blocked but try moving left if clear
    elif not obs_present[3]:
        print "Rolling left because stereo blocked"
        setpoint.roll(2, check_obs=False, wait=True)  # roll left 2m
        usonic.threshold[0] = front_threshold + front_threshold_delta  # set front sensor threshold to 5M
        time.sleep(2)
        if not (obs_present[0]):
            print "Front sensor clear, reading is %s" % usonic.readings[0]
            print "Pitching forward"
        else:
            print "Front sensor not clear rolling left again"
            setpoint.roll(-2, check_obs=False, wait=True, delay=0.5)  # roll left 2m
            if obs_present[0]:
                print "Front sensor not clear reading is %s, regain manual control" % usonic.readings[0]
                trapped = True

    # stereo blocked but try moving right if clear
    elif not obs_present[1]:
        print "Rolling right because stereo blocked"
        setpoint.roll(-2, check_obs=False, wait=True)  # roll right 2m
        usonic.threshold[0] = front_threshold + front_threshold_delta  # set front sensor threshold to 5M
        time.sleep(2)
        if not (obs_present[0]):
            print "Front sensor clear, reading is %s" % usonic.readings[0]
            print "Pitching forward"
        else:
            print "Front sensor not clear rolling right again"
            setpoint.roll(-2, check_obs=False, wait=True, delay=0.5)  # roll right 2m
            if obs_present[0]:
                print "Front sensor not clear reading is %s, regain manual control" % usonic.readings[0]
                trapped = True

    else:
        print "No clear path, regain manual control"
        trapped = True

    usonic.threshold[0] = front_threshold # set front sensor threshold to 5M

#initialize sonar sensor network and vision system
stereo = Stereo_Vision(cam_L_src=0,cam_R_src=1, display_frames=False)
usonic = usonic(0x70, 4, disable_mask = [False, True, True, True, True, True], sim_readings=[-1,600,600,600,600,600], threshold=[front_threshold,450,450,450,450])

#started threaded sonar sensor and stereo vision system classes
usonic.start()
stereo.start()

#initailize ROS publisher for setpoints at 10Hz
rospy.init_node('setpoint_position_demo')
mavros.set_namespace()  # initialize mavros module with default namespace
rate = rospy.Rate(10)
setpoint = Setpoint(sonar_obs_present=usonic.obs_present, jMAVSim = False)

#delay for webcams to wake-up
time.sleep(5)

set_home = raw_input("Press 'h' to set home 2M above current location")

if set_home == 'h':
    setpoint.set_home(alt=2.3)

raw_input("Press any key to begin")

#pitch forward 15M while checking for obstacles
setpoint.pitch(15, wait=False, check_obs=True, store_final = True)

while setpoint.done == False and setpoint.obs_detected == False:
    pass

if setpoint.obs_detected == True:
    setpoint.obs_detected = False

    usonic.disable_mask = [False, False, False, False, True, True]
    time.sleep(2)
    #decide best navigation from sensor/camera readings
    nav_decision(usonic.obs_present, stereo.no_object_bottom_row, stereo.no_object_mid_row, stereo.no_object_top_row, stereo.no_object_left_column, stereo.no_object_mid_column, stereo.no_object_right_column)

    if not(trapped):
        #if drone didn't encounter trapped condition, pitch forward 4m to clear obstacle
        usonic.disable_mask = [False, True, True, True, True, True]
        setpoint.pitch(4, wait=False, check_obs=True, store_final=False)

        while setpoint.done == False and setpoint.obs_detected == False:
            pass

        #while pitching past obstacle, drone encountered second obstacle
        if setpoint.obs_detected == True:
            setpoint.obs_detected = False
            usonic.disable_mask = [False, False, False, False, True, True]
            time.sleep(2)

            # decide best navigation path from sensor/camera readings
            nav_decision(usonic.obs_present, stereo.no_object_bottom_row, stereo.no_object_mid_row,
                         stereo.no_object_top_row, stereo.no_object_left_column, stereo.no_object_mid_column,
                         stereo.no_object_right_column)

            if not(trapped):
                usonic.disable_mask = [False, True, True, True, True, True]
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
        time.sleep(0.5)

        #navigate to final destination
        setpoint.set(setpoint.x_final_dest,setpoint.y_final_dest,setpoint.z_final_dest,setpoint.delta_yaw)

else:
    print "No obstacle detected along path"

print "Final destination reached"

#yaw toward start point
print "Returning to home"
print setpoint.x_home, setpoint.y_home

setpoint.yaw_towards_dest(setpoint.x_home, setpoint.y_home)
time.sleep(5)

usonic.disable_mask = [False, True, True, True, True, True]
setpoint.pitch(15, wait=False, check_obs=True, store_final=True)

if setpoint.obs_detected == True:
    setpoint.obs_detected = False
    usonic.disable_mask = [False, False, False, False, True, True]
    time.sleep(2)

    # decide best navigation from sensor/camera readings
    nav_decision(usonic.obs_present, stereo.no_object_bottom_row, stereo.no_object_mid_row, stereo.no_object_top_row, stereo.no_object_left_column, stereo.no_object_mid_column, stereo.no_object_right_column)

    if not (trapped):
        # if drone didn't encounter trapped condition, pitch forward 4m to clear obstacle
        usonic.disable_mask = [False, True, True, True, True, True]
        setpoint.pitch(4, wait=False, check_obs=True, store_final=False)

        while setpoint.done == False and setpoint.obs_detected == False:
            pass

        # while pitching past obstacle, drone encountered second obstacle
        if setpoint.obs_detected == True:
            setpoint.obs_detected = False
            usonic.disable_mask = [False, False, False, False, True, True]
            time.sleep(2)

            # decide best navigation path from sensor/camera readings
            nav_decision(usonic.obs_present, stereo.no_object_bottom_row, stereo.no_object_mid_row,
                         stereo.no_object_top_row, stereo.no_object_left_column, stereo.no_object_mid_column,
                         stereo.no_object_right_column)

            if not (trapped):
                usonic.disable_mask = [False, True, True, True, True, True]
                setpoint.pitch(4, wait=False, check_obs=True, store_final=False)

                while setpoint.done == False and setpoint.obs_detected == False:
                    pass

                if setpoint.obs_detected:
                    setpoint.obs_detected = False
                    print "On second attempt past obstacle, drone encountered obstacle, halting, regain manual control"
                    trapped = True

    if not(trapped):
        setpoint.yaw_towards_dest(setpoint.x_final_dest, setpoint.y_final_dest)
        time.sleep(0.5)

        #navigate to home
        setpoint.set(setpoint.x_final_dest,setpoint.y_final_dest,setpoint.curr_z,setpoint.delta_yaw)


#yaw back to 0 degrees
setpoint.yaw_abs(0)

print "Mission complete"
