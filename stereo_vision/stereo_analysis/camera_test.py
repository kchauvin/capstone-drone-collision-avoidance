import cv2
import sys
import time
import numpy as np
from imutils.video import WebcamVideoStream
import imutils

#from matplotlib import pyplot as plt

def nothing(x):
    pass

# grab camera feeds
#capL = cv2.VideoCapture(1)
#capR = cv2.VideoCapture(0)

capL = WebcamVideoStream(src=1, width=640, height=480).start()
capR = WebcamVideoStream(src=0, width=640, height=480).start()

#capL.set(3,640)
#capL.set(4,480)


# initialize windows
WINDOW_L = 'undistorted left cam'
WINDOW_R = 'undistorted right cam'
#cv2.namedWindow('Depth Map')
#cv2.namedWindow('Threshold')
#cv2.namedWindow('Shapes')
#cv2.namedWindow('Test')



# import calibration matrices
undistMapL = np.load('calibration/undistortion_map_left.npy')
undistMapR = np.load('calibration/undistortion_map_right.npy')
rectMapL = np.load('calibration/rectification_map_left.npy')
rectMapR = np.load('calibration/rectification_map_right.npy')


#initialize blank frames


exec_time_sum = 0
i = 0



while (True):

    start_time = time.time()
    frameL = capL.read()
    frameR = capR.read()

    end_time = time.time()
    exec_time = end_time - start_time
    exec_time_sum += exec_time
    i += 1
    fps = 1.0/exec_time

    cv2.imshow(WINDOW_L, frameL)
    cv2.imshow(WINDOW_R, frameR)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        capL.stop()
        capR.stop()
        break


capL.stop()
capR.stop()
cv2.destroyAllWindows()

exec_time_avg = exec_time_sum/i
fps_avg = 1.0/exec_time_avg

print "Average Execution Time/Frame : %s" %exec_time_avg
print "Average FPS : %s" %fps_avg
