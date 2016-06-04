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

capL = WebcamVideoStream(src=1).start()
capR = WebcamVideoStream(src=0).start()


# initialize windows
WINDOW_L1 = 'undistorted left cam'
WINDOW_R1 = 'undistorted right cam'
cv2.namedWindow('Depth Map')
cv2.namedWindow('Threshold')
cv2.namedWindow('Shapes')
cv2.namedWindow('Test')



# import calibration matrices
undistMapL = np.load('calibration/undistortion_map_left.npy')
undistMapR = np.load('calibration/undistortion_map_right.npy')
rectMapL = np.load('calibration/rectification_map_left.npy')
rectMapR = np.load('calibration/rectification_map_right.npy')


#initialize blank frames
rectL = np.zeros((640,480,3),np.uint8)
rectR = np.zeros((640,480,3),np.uint8)




exec_time_sum = 0
i = 0


while (True):

    start_time = time.time()
    frameL = capL.read()
    frameR = capR.read()


    #remap cameras to remove distortion
    rectL = cv2.remap(frameL, undistMapL, rectMapL, cv2.INTER_LINEAR)
    rectR = cv2.remap(frameR, undistMapR, rectMapR, cv2.INTER_LINEAR)


    # convert rectified from RGB to 8 bit grayscale
    grayRectL = cv2.cvtColor(rectL, cv2.COLOR_BGR2GRAY)
    grayRectR = cv2.cvtColor(rectR, cv2.COLOR_BGR2GRAY)
    grayFrameL = cv2.cvtColor(frameL, cv2.COLOR_BGR2GRAY)
    grayFrameR = cv2.cvtColor(frameR, cv2.COLOR_BGR2GRAY)

    #create depth map

    window_size = 3
    min_disp = 16
    num_disp = 112 - min_disp
    stereo = cv2.StereoSGBM_create(
        minDisparity = min_disp,
        numDisparities = num_disp,
        blockSize = 27,
        P1 = 8*3*window_size**2,
        P2 = 32*3*window_size**2,
        disp12MaxDiff = 10,
        uniquenessRatio = 0,
        speckleWindowSize = 0,
        speckleRange = 0,
        preFilterCap=0
    )


    disparity = stereo.compute(grayRectL,grayRectR).astype(np.float32)
    disparity = cv2.normalize(disparity, disparity, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)


    #blur depth map to filter high frequency noise
    #disparity_gaussian = cv2.GaussianBlur(disparity,(9,9),0)
    disparity_blur = cv2.medianBlur(disparity,11)

    #thresholding depth map
    disparity_thresh_near = cv2.threshold(disparity_blur,170,255,cv2.THRESH_BINARY)[1]
    disparity_thresh_imm = cv2.threshold(disparity_blur,180,255,cv2.THRESH_BINARY)[1]
    disparity_thresh_near = cv2.subtract(disparity_thresh_near, disparity_thresh_imm)


    disparity_edge = cv2.Canny(disparity_thresh_imm,100,200)

    #contour finding
    #contours = cv2.findContours(disparity_thresh_imm, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[1]
    contours = cv2.findContours(disparity_edge, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[1]

    disparity_contours = np.zeros((480,640,3),np.uint8)

    min_box_size = 200 #pixels^2
    quadrant_near_object = np.zeros((3,3), dtype=bool)

    for contour_index in contours:
        area = cv2.contourArea(contour_index)

        rect = cv2.minAreaRect(contour_index)
        box = cv2.boxPoints(rect)
        box_area = abs((box[0][0] - box[1][0]) * (box[0][1] - box[1][1]))
        box = np.int0(box)

        if box_area > min_box_size:
            cv2.drawContours(disparity_contours, [box], 0, (0,255,0), 2)

            #determine which quadrants the rectangle occupies
            for i in range(0,1):
                for j in range(0,1):
                    if box[i][j] < (213,160):
                        quadrant_near_object[0][0] = True
                    elif (213,0) <= box[i][j] < (426,0):
                        quadrant_near_object[1][0] = True
                    elif (426,0) <= box[i][j] < (640,0):
                        quadrant_near_object[2][0] = True
                    elif (0,160) <= box[i][j] < (0,320):
                        quadrant_near_object[0][1] = True
                    elif (213,160) <= box[i][j] < (213,320):
                        quadrant_near_object[1][1] = True
                    elif (426,160) <= box[i][j] < (640,320):
                        quadrant_near_object[2][1] = True
                    elif (0,320) <= box[i][j] < (0,480):
                        quadrant_near_object[0][1] = True
                    elif (213,320) <= box[i][j] < (213,480):
                        quadrant_near_object[1][1] = True
                    elif (426,320) <= box[i][j] < (640,480):
                        quadrant_near_object[2][1] = True

        # M  = cv2.moments(contours[i])
        # if (M['m00'] == 0):
        #       M['m00'] = 1
        #  if M['m00'] != 0:
        #     cx = int(M['m10']/M['m00'])
        #      cy = int(M['m01']/M['m00'])
        #      cv2.circle(disparity_contours, (cx, cy), 7, (255, 255, 255), -1)
        #      cv2.putText(disparity_contours, "center", (cx - 20, cy - 20),
        #      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    cv2.drawContours(disparity_contours, contours, -1, (180,105,255), -1)


    end_time = time.time()
    exec_time = end_time - start_time
    exec_time_sum += exec_time
    i += 1
    fps = 1.0/exec_time

    #cv2.imshow(WINDOW_L, frameL)
    #cv2.imshow(WINDOW_R, frameR)
    #cv2.imshow(WINDOW_L1, rectL)
    #cv2.imshow(WINDOW_R1, rectR)
    cv2.imshow('Threshold', disparity_thresh_imm)
    cv2.imshow('Depth Map', disparity)
    cv2.imshow('Shapes', disparity_contours)
    #cv2.imshow('Test', disparity_gaussian)

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
