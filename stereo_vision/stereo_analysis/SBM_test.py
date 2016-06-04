import cv2
import sys
import time
import numpy as np
from matplotlib import pyplot as plt

#num disparities = 7*16


def nothing(x):
    pass


# grab camera feeds
capL = cv2.VideoCapture(1)
capR = cv2.VideoCapture(0)

#capL.set(3,1280)
#capL.set(4,720)


# initialize windows
WINDOW_L = 'input left cam'
WINDOW_R = 'input right cam'
WINDOW_L1 = 'undistorted left cam'
WINDOW_R1 = 'undistorted right cam'
cv2.namedWindow('Depth Map')

# import calibration matrices (NOT NEEDED)
camMatL = np.load('calibration/cam_mats_left.npy')
distCoL = np.load('calibration/dist_coefs_left.npy')
camMatR = np.load('calibration/cam_mats_right.npy')
distCoR = np.load('calibration/dist_coefs_right.npy')
R = np.load('calibration/rot_mat.npy')
T = np.load('calibration/trans_vec.npy')
rectTransL = np.load('calibration/rect_trans_left.npy')
rectTransR = np.load('calibration/rect_trans_right.npy')
projMatsL = np.load('calibration/proj_mats_left.npy')
projMatsR = np.load('calibration/proj_mats_right.npy')

print distCoL

# import calibration matricies
undistMapL = np.load('calibration/undistortion_map_left.npy')
undistMapR = np.load('calibration/undistortion_map_right.npy')
rectMapL = np.load('calibration/rectification_map_left.npy')
rectMapR = np.load('calibration/rectification_map_right.npy')

print undistMapL
print "test"
print undistMapR


# output matrices from stereoRectify init
R1 = np.zeros((3, 3))
R2 = np.zeros((3, 3))
P1 = np.zeros((3, 4))
P2 = np.zeros((3, 4))
Q = np.zeros((4, 4))
newImSize = np.zeros((2))

# new camera matrixes from undistortrectifymap
newCamMatL = np.zeros((3, 4))
newCamMatR = np.zeros((3, 4))

# camera rectification maps initialization
mapxL = np.zeros((3, 3))
mapyL = np.zeros((3, 3))
mapxR = np.zeros((3, 3))
mapyR = np.zeros((3, 3))

# new rectified images initialization
rectL = np.zeros((640, 480, 3), np.uint8)
rectR = np.zeros((640, 480, 3), np.uint8)

prefiltercap = 0
# test sliders for tuning SGBM
cv2.createTrackbar('numDisp*16', 'Depth Map', 1, 100, nothing)
cv2.createTrackbar('BlockSize', 'Depth Map', 1, 100, nothing)
cv2.setTrackbarPos('numDisp*16', 'Depth Map', 1)
cv2.setTrackbarPos('BlockSize', 'Depth Map', 1)

exec_time_sum = 0
i = 0

# image size
imSize = (640, 480)

while (True):

    start_time = time.time()
    retL, frameL = capL.read()
    retR, frameR = capR.read()

    # remap cameras to remove distortion
    # note can convert from floats to int later using convertMaps() for faster speed
    rectL = cv2.remap(frameL, undistMapL, rectMapL, cv2.INTER_LINEAR)
    rectR = cv2.remap(frameR, undistMapR, rectMapR, cv2.INTER_LINEAR)

    # Depth Map
    # convert rectified from RGB to 8 bit grayscale
    grayRectL = cv2.cvtColor(rectL, cv2.COLOR_BGR2GRAY)
    grayRectR = cv2.cvtColor(rectR, cv2.COLOR_BGR2GRAY)
    grayFrameL = cv2.cvtColor(frameL, cv2.COLOR_BGR2GRAY)
    grayFrameR = cv2.cvtColor(frameR, cv2.COLOR_BGR2GRAY)

    # create depth map WITH sliders
    num_disp = 16 * cv2.getTrackbarPos('numDisp*16', 'Depth Map')
    blocksize = 3 + 2 * cv2.getTrackbarPos('BlockSize', 'Depth Map')

    stereo = cv2.StereoBM_create(
        numDisparities=num_disp,
        blockSize=blocksize)

    disparity = stereo.compute(grayRectL, grayRectR).astype(np.float32)
    disparity = cv2.normalize(disparity, disparity, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    # fps calculations
    end_time = time.time()
    exec_time = end_time - start_time
    exec_time_sum += exec_time
    i += 1
    fps = 1.0 / exec_time

    # cv2.imshow(WINDOW_L, frameL)
    # cv2.imshow(WINDOW_R, frameR)

    cv2.imshow(WINDOW_L1, rectL)
    cv2.imshow(WINDOW_R1, rectR)
    cv2.imshow('Depth Map', disparity)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

capL.release()
cv2.destroyAllWindows()

exec_time_avg = exec_time_sum / i
fps_avg = 1.0 / exec_time_avg

print "Average Execution Time/Frame : %s" % exec_time_avg
print "Average FPS : %s" % fps_avg
