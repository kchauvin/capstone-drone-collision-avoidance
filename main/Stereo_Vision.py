import cv2
import sys
import time
from threading import Thread
import numpy as np
from imutils.video import WebcamVideoStream


class Stereo_Vision:
    """Class for obtaining and analyzing depth maps"""

    def __init__(self, cam_L_src, cam_R_src, display_frames=True, threshold=110):

        # initialize camera feed threads
        self.capL = WebcamVideoStream(src=cam_L_src).start()
        time.sleep(0.5)
        self.capR = WebcamVideoStream(src=cam_R_src).start()

        self.stop = False

        self.display_frames = display_frames

        # initialize windows
        if self.display_frames == True:
            cv2.namedWindow('Depth Map')
            cv2.namedWindow('Threshold')
            cv2.namedWindow('Shapes')

        # import calibration matrices
        self.undistMapL = np.load('calibration/undistortion_map_left.npy')
        self.undistMapR = np.load('calibration/undistortion_map_right.npy')
        self.rectMapL = np.load('calibration/rectification_map_left.npy')
        self.rectMapR = np.load('calibration/rectification_map_right.npy')

        # initialize blank frames
        self.rectL = np.zeros((640, 480, 3), np.uint8)
        self.rectR = np.zeros((640, 480, 3), np.uint8)

        self.quadrant_near_object = np.zeros((3, 3), dtype=bool)
        self.threshold = threshold

        self.exec_time_sum = 0
        self.frame_counter = 0
        self.fps = 0

        self.no_object_left_column = False
        self.no_object_mid_column = False
        self.no_object_right_column = False
        self.no_object_bottom_row = False
        self.no_object_mid_row = False
        self.no_object_top_row = False

    def start(self):
        self.vision_thread = Thread(target=self.start_stereo)
        self.vision_thread.start()

    def stop_vision(self):
        self.stop = True

    def get_quadrants(self):
        return self.quadrant_near_object

    def start_stereo(self):
        while (self.stop != True):

            self.start_time = time.time()
            self.frameL = self.capL.read()
            self.frameR = self.capR.read()

            # remap cameras to remove distortion
            self.rectL = cv2.remap(self.frameL, self.undistMapL, self.rectMapL, cv2.INTER_LINEAR)
            self.rectR = cv2.remap(self.frameR, self.undistMapR, self.rectMapR, cv2.INTER_LINEAR)

            # convert rectified from RGB to 8 bit grayscale
            self.grayRectL = cv2.cvtColor(self.rectL, cv2.COLOR_BGR2GRAY)
            self.grayRectR = cv2.cvtColor(self.rectR, cv2.COLOR_BGR2GRAY)
            self.grayFrameL = cv2.cvtColor(self.frameL, cv2.COLOR_BGR2GRAY)
            self.grayFrameR = cv2.cvtColor(self.frameR, cv2.COLOR_BGR2GRAY)

            # create depth map

            self.object_left_column = True
            self.object_mid_column = True
            self.object_right_column = True
            self.object_top_row = True
            self.object_mid_row = True
            self.object_bottom_row = True

            self.stereo = cv2.StereoBM_create(numDisparities=0, blockSize=13)

            self.disparity = self.stereo.compute(self.grayRectL, self.grayRectR).astype(np.float32)
            self.disparity = cv2.normalize(self.disparity, self.disparity, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX,
                                           dtype=cv2.CV_8UC1)

            # blur depth map to filter high frequency noise
            self.disparity_blur = cv2.medianBlur(self.disparity, 13)

            #apply 2 pixel border, helps keep contours continuous at extreme edges of image
            self.disparity_blur = cv2.copyMakeBorder(self.disparity_blur, top=2, bottom=2, right=2, left=2,
                                                     borderType=cv2.BORDER_CONSTANT, value=0)

            # threshold depth map
            self.disparity_thresh_imm = cv2.threshold(self.disparity_blur, self.threshold, 255, cv2.THRESH_BINARY)[1]

            self.disparity_edge = cv2.Canny(self.disparity_thresh_imm, 100, 200)

            # contour finding
            self.contours = cv2.findContours(self.disparity_edge, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[1]

            self.disparity_contours = np.zeros((480, 640, 3), np.uint8)

            for self.contour_index in self.contours:
                #draw bounding rectangle around each contour
                self.rect = cv2.minAreaRect(self.contour_index)
                self.box = cv2.boxPoints(self.rect)
                self.box = np.int0(self.box)

                #calculate area of each box
                self.box_area = abs(self.box[2][0] - self.box[0][0]) * abs(self.box[2][1] - self.box[0][1])
                self.col = -1
                self.row = -1

                if self.box_area > 1000: #pixels^2
                    cv2.drawContours(self.disparity_contours, [self.box], 0, (0, 255, 0), 2)  # draw green box

                    # determine which quadrants the rectangle occupies
                    for corner_index in range(0, 3):
                        self.col = self.box[corner_index][0] / 213  # x coordinates
                        self.row = self.box[corner_index][1] / 160  # y coordinates
                        if self.row > 2:
                            self.row = 2

                        if self.col > 2:
                            self.col = 2

                        if self.col != -1 and self.row != -1:
                            self.quadrant_near_object[self.row][self.col] = True


            #Boolean logic to set flags for main program
            if (self.quadrant_near_object[0][0] == False and self.quadrant_near_object[1][0] == False or
                        self.quadrant_near_object[2][0] == False):
                self.no_object_left_column = True
            else:
                self.no_object_left_column = False

            if (self.quadrant_near_object[0][1] == False and self.quadrant_near_object[1][1] == False and
                        self.quadrant_near_object[2][1] == False):
                self.no_object_mid_column = True
            else:
                self.no_object_mid_column = False

            if (self.quadrant_near_object[0][2] == False and self.quadrant_near_object[1][2] == False and
                        self.quadrant_near_object[2][2] == False):
                self.no_object_right_column = True
            else:
                self.no_object_right_column = False

            if (self.quadrant_near_object[0][0] == False and self.quadrant_near_object[0][1] == False and
                        self.quadrant_near_object[0][2] == False):
                self.no_object_top_row = True
            else:
                self.no_object_top_row = False
            if (self.quadrant_near_object[1][0] == False and self.quadrant_near_object[1][1] == False and
                        self.quadrant_near_object[1][2] == False):
                self.no_object_mid_row = True
            else:
                self.no_object_mid_row = False

            if (self.quadrant_near_object[2][0] == False and self.quadrant_near_object[2][1] == False and
                        self.quadrant_near_object[2][2] == False):
                self.no_object_bottom_row = True
            else:
                self.no_object_bottom_row = False

            #FPS calculations
            self.end_time = time.time()
            self.exec_time = self.end_time - self.start_time
            self.exec_time_sum += self.exec_time
            self.frame_counter += 1
            self.fps = 1.0 / self.exec_time

            #draw edges in pink
            cv2.drawContours(self.disparity_contours, self.contours, -1, (180, 105, 255), -1)
            cv2.line(self.disparity_contours, (0,160), (640,160), (255,0,0))
            cv2.line(self.disparity_contours, (0,320), (640,320), (255,0,0))
            cv2.line(self.disparity_contours, (213,0), (213,480), (255,0,0))
            cv2.line(self.disparity_contours, (426,0), (426,480), (255,0,0))


            if self.display_frames == True:
                cv2.imshow('Threshold', self.disparity_edge)
                cv2.imshow('Depth Map', self.disparity)
                cv2.imshow('Shapes', self.disparity_contours)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.capL.stop()
                self.capR.stop()
                break

        self.capL.stop()
        self.capR.stop()
        cv2.destroyAllWindows()

    def save_frames(self, filename_index):
        cv2.imwrite("Images/DepthMap" + str(filename_index) + ".jpg", self.disparity)
        cv2.imwrite("Images/DepthMapBlur" + str(filename_index) + ".jpg", self.disparity_blur)
        cv2.imwrite("Images/Contours_" + str(filename_index) + ".jpg", self.disparity_contours)
        cv2.imwrite("Images/LeftCam" + str(filename_index) + ".jpg", self.frameL)
        cv2.imwrite("Images/RightCam" + str(filename_index) + ".jpg", self.frameR)







