import cv2
import sys
import time
import numpy as np
from imutils.video import WebcamVideoStream
import imutils

from matplotlib import pyplot as plt

def nothing(x):
    pass

# grab camera feeds
#capL = cv2.VideoCapture(1)
#capR = cv2.VideoCapture(0)

capL = WebcamVideoStream(src=0).start()
capR = WebcamVideoStream(src=1).start()


# initialize windows
WINDOW_L1 = 'undistorted left cam'
WINDOW_R1 = 'undistorted right cam'
cv2.namedWindow('Depth Map')
cv2.namedWindow('Threshold')



# import calibration matrices
undistMapL = np.load('calibration/undistortion_map_left.npy')
undistMapR = np.load('calibration/undistortion_map_right.npy')
rectMapL = np.load('calibration/rectification_map_left.npy')
rectMapR = np.load('calibration/rectification_map_right.npy')


#new rectified images initialization
rectL = np.zeros((640,480,3),np.uint8)
rectR = np.zeros((640,480,3),np.uint8)



#test sliders for tuning SGBM
cv2.createTrackbar('minDisp*16','Depth Map',1,100,nothing)
cv2.createTrackbar('numDisp*16','Depth Map',1,100,nothing)
cv2.createTrackbar('UniquenessRatio','Depth Map',1,100,nothing)
cv2.createTrackbar('SpeckleWinSize','Depth Map',1,100,nothing)
cv2.createTrackbar('BlockSize','Depth Map',1,100,nothing)
cv2.setTrackbarPos('minDisp*16','Depth Map',1)
cv2.setTrackbarPos('numDisp*16','Depth Map',1)
cv2.setTrackbarPos('UniquenessRatio','Depth Map',1)
cv2.setTrackbarPos('SpeckleWinSize','Depth Map',1)
cv2.setTrackbarPos('BlockSize','Depth Map',1)

#thresholding trackbar
cv2.createTrackbar('Threshold','Depth Map',0,255,nothing)
cv2.setTrackbarPos('Threshold','Depth Map',128)


exec_time_sum = 0
i = 0


#blob detection parameters
blob_params = cv2.SimpleBlobDetector_Params()

blob_params.minThreshold = 0
blob_params.thresholdStep = 10
blob_params.maxThreshold = 120
blob_params.filterByArea = True
blob_params.minArea = 100
blob_params.filterByColor = False


blob_params.minDistBetweenBlobs = 50


#filter by size
#blob_params.filterByArea = True
#blob_params.minArea = 250

blob_detector = cv2.SimpleBlobDetector_create(parameters=blob_params)


while (True):

    start_time = time.time()
    frameL = capL.read()
    frameR = capR.read()


    #remap cameras to remove distortion
    rectL = cv2.remap(frameL, undistMapL, rectMapL, cv2.INTER_LINEAR)
    rectR = cv2.remap(frameR, undistMapR, rectMapR, cv2.INTER_LINEAR)


    #~~~~ DEPTH MAP PART ~~~~~~~
    # convert rectified from RGB to 8 bit grayscale
    grayRectL = cv2.cvtColor(rectL, cv2.COLOR_BGR2GRAY)
    grayRectR = cv2.cvtColor(rectR, cv2.COLOR_BGR2GRAY)
    grayFrameL = cv2.cvtColor(frameL, cv2.COLOR_BGR2GRAY)
    grayFrameR = cv2.cvtColor(frameR, cv2.COLOR_BGR2GRAY)

    #create depth map with sliders
    min_disp= 16*cv2.getTrackbarPos('minDisp*16','Depth Map')
    num_disp = 16*cv2.getTrackbarPos('numDisp*16','Depth Map')
    blocksize = cv2.getTrackbarPos('BlockSize','Depth Map')
    unique_ratio = cv2.getTrackbarPos('UniquenessRatio','Depth Map')
    speckle_win_size = cv2.getTrackbarPos('SpeckleWinSize','Depth Map')

    window_size = 3
    min_disp = 16
    num_disp = 64 - min_disp
    stereo = cv2.StereoSGBM_create(
        minDisparity = min_disp,
        numDisparities = num_disp,
        blockSize = 22,
        P1 = 8*3*window_size**2,
        P2 = 32*3*window_size**2,
        disp12MaxDiff = 10,
        uniquenessRatio = 0,
        speckleWindowSize = 0,
        speckleRange = 0,
        preFilterCap=0
    )


    disparity = stereo.compute(grayRectL,grayRectR).astype(np.float32)
    disparity = cv2.normalize(disparity, disparity, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    #~~~~~~ END DEPTH MAP ~~~~~~~~


    # image size


    #threshold


    threshold_value = cv2.getTrackbarPos('Threshold', 'Depth Map')

    #disparity_threshold = cv2.threshold(disparity, threshold_value, 255, cv2.THRESH_BINARY_INV)[1]

    disparity_inverted = cv2.subtract(255, disparity)

    #blobs
    keypoints = blob_detector.detect(disparity)
    frame_with_keypoints = cv2.drawKeypoints(disparity,keypoints,np.array([]),(0.0,255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)


    end_time = time.time()
    exec_time = end_time - start_time
    exec_time_sum += exec_time
    i += 1
    fps = 1.0/exec_time

    #cv2.imshow(WINDOW_L, frameL)
    #cv2.imshow(WINDOW_R, frameR)
    #cv2.imshow(WINDOW_L1, rectL)
    cv2.imshow(WINDOW_R1, rectR)
    #cv2.imshow('Threshold', disparity)
    cv2.imshow('Depth Map', frame_with_keypoints)


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
