import cv2
import sys
import time
import threading
import numpy as np
from imutils.video import WebcamVideoStream

from matplotlib import pyplot as plt

#def nothing(x):
#    pass


class Stereo_Vision:
    """Class for obtaining and analyzing depth maps"""

    def __init__(self, cam_L_src, cam_R_src, display_frames=True):

        #initialize camera feed threads
        self.capL = WebcamVideoStream(src=cam_L_src).start()
        self.capR = WebcamVideoStream(src=cam_R_src).start()

        self.display_frames = display_frames

        # initialize windows
        self.WINDOW_L1 = 'undistorted left cam'
        self.WINDOW_R1 = 'undistorted right cam'
        cv2.namedWindow('Depth Map')
        cv2.namedWindow('Threshold')
        cv2.namedWindow('Shapes')
        cv2.namedWindow('Test')


        # import calibration matrices
        self.undistMapL = np.load('calibration/undistortion_map_left.npy')
        self.undistMapR = np.load('calibration/undistortion_map_right.npy')
        self.rectMapL = np.load('calibration/rectification_map_left.npy')
        self.rectMapR = np.load('calibration/rectification_map_right.npy')


        #initialize blank frames
        self.rectL = np.zeros((640,480,3),np.uint8)
        self.rectR = np.zeros((640,480,3),np.uint8)


        self.exec_time_sum = 0
        self.frame_counter = 0
        self.stop = False

    def start(self):
        self.vision_thread = Thread(target=start_stereo, args=self)
        self.vision_thread.start()

    def stop(self):
        self.stop = True
        self.quadrant_near_object = np.zeros((3,3), dtype=bool)

    def get_quadrants(self):
        return self.quadrant_near_object

    def start_stereo(self):
        while (self.stop != True):
            self.frameL = capL.read()
            self.frameR = capR.read()


            #remap cameras to remove distortion
            self.rectL = cv2.remap(self.frameL, self.undistMapL, self.rectMapL, cv2.INTER_LINEAR)
            self.rectR = cv2.remap(self.frameR, self.undistMapR, self.rectMapR, cv2.INTER_LINEAR)


            # convert rectified from RGB to 8 bit grayscale
            self.grayRectL = cv2.cvtColor(self.rectL, cv2.COLOR_BGR2GRAY)
            self.grayRectR = cv2.cvtColor(self.rectR, cv2.COLOR_BGR2GRAY)
            self.grayFrameL = cv2.cvtColor(self.frameL, cv2.COLOR_BGR2GRAY)
            self.grayFrameR = cv2.cvtColor(selfframeR, cv2.COLOR_BGR2GRAY)

            #create depth map

            self.object_left_column = True
            self.object_mid_column = True
            self.object_right_column = True
            self.object_top_row = True
            self.object_mid_row = True
            self.object_bottom_row = True


            self.window_size = 3
            self.min_disp = 16
            self.num_disp = 112 - min_disp
            self.stereo = cv2.StereoSGBM_create(
                minDisparity = self.min_disp,
                numDisparities = self.num_disp,
                blockSize = 27,
                P1 = 8*3*self.window_size**2,
                P2 = 32*3*self.window_size**2,
                disp12MaxDiff = 10,
                uniquenessRatio = 0,
                speckleWindowSize = 0,
                speckleRange = 0,
                preFilterCap=0
            )


            self.disparity = stereo.compute(self.grayRectL,self.grayRectR).astype(np.float32)
            self.disparity = cv2.normalize(self.disparity, self.disparity, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)


            #blur depth map to filter high frequency noise
            #disparity_gaussian = cv2.GaussianBlur(disparity,(9,9),0)
            self.disparity_blur = cv2.medianBlur(self.disparity,11)

            #thresholding depth map
            self.disparity_thresh_near = cv2.threshold(self.disparity_blur,170,255,cv2.THRESH_BINARY)[1]
            self.disparity_thresh_imm = cv2.threshold(self.disparity_blur,180,255,cv2.THRESH_BINARY)[1]
            disparity_thresh_near = cv2.subtract(self.disparity_thresh_near, self.disparity_thresh_imm)


            self.disparity_edge = cv2.Canny(self.disparity_thresh_imm,100,200)

            #contour finding
            #contours = cv2.findContours(disparity_thresh_imm, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[1]
            self.contours = cv2.findContours(self.disparity_edge, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[1]

            self.disparity_contours = np.zeros((480,640,3),np.uint8)

            self.min_box_size = 200 #pixels^2
            self.quadrant_near_object = np.zeros((3,3), dtype=bool)

            for self.contour_index in self.contours:
                area = cv2.contourArea(self.contour_index)

                self.rect = cv2.minAreaRect(self.contour_index)
                self.box = cv2.boxPoints(self.rect)
                #this needs to be fixed
                self.box_area = abs((self.box[0][0] - self.box[1][0]) * (self.box[0][1] - self.box[1][1]))
                self.box = np.int0(self.box)

                if self.box_area > self.min_box_size:
                    cv2.drawContours(self.disparity_contours, [self.box], 0, (0,255,0), 2)

                    #determine which quadrants the rectangle occupies
                    #NEED TO TEST THIS WITH THE CAMERAS
                    for row_index in range(0,1):
                        for col_index in range(0,1):
                            self.col = self.box[row_index][col_index][0] / 214
                            self.row = self.box[row_index][col_index][1] / 160
                        if row == 3:
                            row = 2

            if (self.quadrant_near_object[0][0] == True or self.quadrant_near_object[0][1] == True or self.quadrant_near_object[0][2] == True):
                object_left_column = True
            if (self.quadrant_near_object[1][0] == True or self.quadrant_near_object[1][1] == True or self.quadrant_near_object[1][2] == True):
                object_mid_column = True
            if (self.quadrant_near_object[2][0] == True or self.quadrant_near_object[2][1] == True or self.quadrant_near_object[2][2] == True):
                object_right_column = True
            if (self.quadrant_near_object[0][0] == True or self.quadrant_near_object[1][0] == True or self.quadrant_near_object[2][0] == True):
                object_top_row = True
            if (self.quadrant_near_object[0][1] == True or self.quadrant_near_object[1][1] == True or self.quadrant_near_object[2][1] == True):
                object_mid_row = True
            if (self.quadrant_near_object[0][2] == True or self.quadrant_near_object[1][2] == True or self.quadrant_near_object[2][2] == True):
                object_bottom_row = True


            if display_frames == True:
                cv2.drawContours(self.disparity_contours, self.contours, -1, (180,105,255), -1)
                cv2.imshow('Threshold', self.disparity_thresh_imm)
                cv2.imshow('Depth Map', self.disparity)
                cv2.imshow('Shapes', self.disparity_contours)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.capL.stop()
                self.capR.stop()
                break

        self.capL.stop()
        self.capR.stop()
        cv2.destroyAllWindows()



SV = Stereo_Vision(1,0)

SV.start()

SV.stop()


sonar = usonic(;llkadsf)
sonar.start()


print sonar.readings[0]