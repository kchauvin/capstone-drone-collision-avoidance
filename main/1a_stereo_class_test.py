import numpy
import cv2
import sys
import time
import rospy
import thread
import mavros
import numpy as np
import imutils
import os


from threading import Thread
from geometry_msgs.msg import PoseStamped, Quaternion
from math import *
from std_msgs.msg import Header
from std_msgs.msg import String
from mavros.utils import *
from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler

from Stereo_Vision import Stereo_Vision

stereo = Stereo_Vision(cam_L_src=1,cam_R_src=0,display_frames=False)
stereo.start()

time.sleep(10)

print "FPS is %s" %stereo.fps

	


