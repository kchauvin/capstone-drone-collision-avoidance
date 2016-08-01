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
from Usonic import usonic


usonic = usonic(0x70, 4, disable_mask = [False, True, True, True, True, True], sim_readings=[-1,600,600,600,600,600], threshold=[300,400,400,400,400])

usonic.start()

for i in range(0, 10):
	print usonic.readings
	time.sleep(0.5)
	
print "Changing mask"
usonic.disable_mask = [False, False, False, False, False, False]

for k in range(0, 100):
        print usonic.readings
	print usonic.obs_present
        time.sleep(0.1)

print "Putting mask back to original"
usonic.disable_mask = [False, True, True, True, True, True]

for j in range(0, 10):
        print usonic.readings
        time.sleep(0.5)


	
