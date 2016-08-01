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

stereo = Stereo_Vision(cam_L_src=1,cam_R_src=0,display_frames=False)
stereo.start()
time.sleep(2)
stereo.save_frames(1)
print stereo.box
print stereo.quadrant_near_object
print stereo.no_object_left_column
print stereo.no_object_mid_column
print stereo.no_object_right_column
print stereo.no_object_top_row
print stereo.no_object_mid_row
print stereo.no_object_bottom_row

#while 1:
	#print stereo.quadrant_near_object
	#os.system('clear')
	#print stereo.no_object_top_row
#print stereo.no_object_bottom_row

time.sleep(5)

print "FPS is %s" %stereo.fps
stereo.stop_vision()
	


