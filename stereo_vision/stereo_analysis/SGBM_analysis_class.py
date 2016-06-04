import cv2
import sys
import time
from threading import Thread
import numpy as np
from imutils.video import WebcamVideoStream
import imutils

from matplotlib import pyplot as plt

def nothing(x):
    pass


class Stereo_Vision:
    """Class for obtaining and analyzing depth maps"""

    def __init__(self, cam_L_src, cam_R_src):

        #initialize camera feed threads
        self.capL = WebcamVideoStream(src=cam_L_src).start()
        self.capR = WebcamVideoStream(src=cam_R_src).start()


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

    def start(self, display_frames=True):
        vision_thread = Thread(target=start_stereo, args=self,display_frames)
        vision_thread.start()

    def get_FPS(self):
        self.exec_time_avg = self.exec_time_sum/self.frame_counter
        self.fps_avg = 1.0/self.exec_time_avg
        return self.fps_avg

    def stop(self):
        self.stop = True
        self.quadrant_near_object = np.zeros((3,3), dtype=bool)

    def get_quadrants(self):
        return self.quadrant_near_object

    def start_stereo(self, display_frames):
        while (self.stop != True):

            self.start_time = time.time()
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
                    for self.i in range(0,1):
                        for self.j in range(0,1):
                            if self.box[i][j] < (213,160):
                                self.quadrant_near_object[0][0] = True
                            elif (213,0) <= box[i][j] < (426,0):
                                self.quadrant_near_object[1][0] = True
                            elif (426,0) <= box[i][j] < (640,0):
                                self.quadrant_near_object[2][0] = True
                            elif (0,160) <= box[i][j] < (0,320):
                                self.quadrant_near_object[0][1] = True
                            elif (213,160) <= box[i][j] < (213,320):
                                self.quadrant_near_object[1][1] = True
                            elif (426,160) <= box[i][j] < (640,320):
                                self.quadrant_near_object[2][1] = True
                            elif (0,320) <= box[i][j] < (0,480):
                                self.quadrant_near_object[0][1] = True
                            elif (213,320) <= box[i][j] < (213,480):
                                self.quadrant_near_object[1][1] = True
                            elif (426,320) <= box[i][j] < (640,480):
                                self.quadrant_near_object[2][1] = True

            if display_frames == True:
                cv2.drawContours(self.disparity_contours, self.contours, -1, (180,105,255), -1)
                cv2.imshow('Threshold', self.disparity_thresh_imm)
                cv2.imshow('Depth Map', self.disparity)
                cv2.imshow('Shapes', self.disparity_contours)



            if cv2.waitKey(1) & 0xFF == ord('q'):
                capL.stop()
                capR.stop()
                break

            end_time = time.time()
            exec_time = end_time - start_time
            exec_time_sum += exec_time
            self.frame_counter += 1

        capL.stop()
        capR.stop()
        cv2.destroyAllWindows()