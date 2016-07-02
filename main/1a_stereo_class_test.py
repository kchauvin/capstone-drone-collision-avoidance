import numpy
import cv2
import sys
import time
import thread
import numpy as np
import imutils
import os


from math import *

from Stereo_Vision import Stereo_Vision

stereo = Stereo_Vision(cam_L_src=1,cam_R_src=0,display_frames=True)
time.sleep(2)
stereo.start()

time.sleep(10)

print "FPS is %s" %stereo.fps

	


