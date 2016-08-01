import cv2
import sys
import time
import numpy as np
import stereovision
from matplotlib import pyplot as plt



#grab camera feeds
capL = cv2.VideoCapture(0)
capR = cv2.VideoCapture(1)

#initialize windows
WINDOW_L = 'debugWindowLeft'
WINDOW_R = 'debugWindowRight'

s

while (True):
    retL, frameL = capL.read()
    retR, frameR = capR.read()

    #convert frames from RGB to 8 bit grayscale
    grayFrameL = cv2.cvtColor(frameL,cv2.COLOR_BGR2GRAY)
    grayFrameR = cv2.cvtColor(frameR,cv2.COLOR_BGR2GRAY)
    c, r, = grayFrameR.shape

    #stereo disparity computation
    window_size=3
    min_disp=16
    num_disp = 64-min_disp
    stereo = cv2.StereoSGBM_create(minDisparity = min_disp,
        numDisparities = num_disp,
        blockSize = 16,
        P1 = 8*3*window_size**2,
        P2 = 32*3*window_size**2,
        disp12MaxDiff = 1,
        uniquenessRatio = 10,
        speckleWindowSize = 100,
        speckleRange = 32
    )
    disparity = stereo.compute(grayFrameL,grayFrameR).astype(np.float32) / 16.0




    #show stereo in static plot, can remove later
#    plt.imshow(disparity,'gray')
#    plt.show()


    cv2.imshow(WINDOW_R, (disparity-min_disp)/num_disp)
    #cv2.imshow(WINDOW_L, frameL)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


capL.release()
cv2.destroyAllWindows()
