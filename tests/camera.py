import cv, cv2
from matplotlib import pyplot as plot
import numpy as np

CAMERA_INDEX = 0
HUE_MIN = 10
HUE_MAX = 100
PIXEL_WIDTH = 640
PIXEL_HEIGHT = 480
THRESHOLD_PERCENTILE = 90
camera = cv2.VideoCapture(CAMERA_INDEX)
camera.set(cv.CV_CAP_PROP_FRAME_WIDTH, PIXEL_WIDTH)
camera.set(cv.CV_CAP_PROP_FRAME_HEIGHT, PIXEL_HEIGHT)
camera.set(cv.CV_CAP_PROP_SATURATION, 1.0)
camera.set(cv.CV_CAP_PROP_BRIGHTNESS, 0.5)
camera.set(cv.CV_CAP_PROP_CONTRAST, 0.5)
camera.set(cv.CV_CAP_PROP_GAIN, 0.5)
#camera.set(cv.CV_CAP_PROP_EXPOSURE, 0.5)
#camera.set(cv.CV_CAP_PROP_WHITE_BALANCE, 0.5)

while True:
    try:
        (s, bgr) = camera.read()
        bgr = np.rot90(bgr)
        if s:
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
            
            hue_min = HUE_MIN
            hue_max = HUE_MAX
            sat_min = np.percentile(hsv[:,:,1], 50)
            sat_max = 255
            val_min = np.percentile(hsv[:,:,2], 50)
            val_max = 255
            threshold_min = np.array([hue_min, sat_min, val_min], np.uint8)
            threshold_max = np.array([hue_max, sat_max, val_max], np.uint8)
            mask = cv2.inRange(hsv, threshold_min, threshold_max)            
            #mask = cv2.erode(mask, np.ones((3,3), np.uint8), iterations =2)
            #mask = cv2.dilate(mask, np.ones((3,3), np.uint8), iterations =1)
            column_sum = mask.sum(axis=0) # vertical summation            
            centroid = int(np.argmax(column_sum))
            egi = np.dstack((mask, mask, mask))
            bgr[:,centroid-1:centroid+1,:] = 0
            egi[:,centroid-1:centroid+1,:] = 255
            hsv[:,centroid-1:centroid+1,:] = 255
            bgr[:,320,:] = 255
            egi[:,320,:] = 255
            hsv[:,320,:] = 255
            output = np.hstack((bgr, egi))
            cv2.imshow('', output)
            if cv2.waitKey(5) == 27:
                pass
            print(centroid)
    except Exception as error:
        print('ERROR: %s' % str(error))
        break
