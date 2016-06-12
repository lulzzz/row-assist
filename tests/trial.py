import cv, cv2
from matplotlib import pyplot as plot
import numpy as np
from datetime import datetime
import serial
import threading
import ast
import os
import time

CAMERA_INDEX = 0
HUE_MIN = 60
HUE_MAX = 180
SAT_PMIN = 33
SAT_PMAX = 100
VAL_PMIN = 5
VAL_PMAX = 95
PIXEL_WIDTH = 640
PIXEL_HEIGHT = 480
THRESHOLD_PERCENTILE = 90
DEVICE = "/dev/ttyACM0"
BAUD = 9600
OUTPUT_DIR = 'output'

camera = cv2.VideoCapture(CAMERA_INDEX)
camera.set(cv.CV_CAP_PROP_FRAME_WIDTH, PIXEL_WIDTH)
camera.set(cv.CV_CAP_PROP_FRAME_HEIGHT, PIXEL_HEIGHT)
camera.set(cv.CV_CAP_PROP_SATURATION, 1.0)
camera.set(cv.CV_CAP_PROP_BRIGHTNESS, 0.5)
camera.set(cv.CV_CAP_PROP_CONTRAST, 0.7)
filename = datetime.strftime(datetime.now(), "%Y-%m-%m %H_%M_%S.csv")
filepath = os.path.join(OUTPUT_DIR, filename)
csvfile = open(filepath, 'w')
arduino = serial.Serial(DEVICE, BAUD)

class Trial:
    def __init__(self):
        self.angle = 0
        self.encoder = 0
        self.offset = 0
        self.run_threads = True

    def update_arduino(self):
        while self.run_threads:
            try:
                ser_data = arduino.readline()
                data = ast.literal_eval(ser_data)                    
                self.angle = data['ang']
                self.encoder = data['enc']
            except Exception:
                pass

    def run(self):
        threading.Thread(target=self.update_arduino, args=(), kwargs={}).start()
        while self.run_threads:
            try:
                (s, bgr) = camera.read()
                if s:
                    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
                    hue_min = HUE_MIN
                    hue_max = HUE_MAX
                    sat_min = np.percentile(hsv[:,:,1], SAT_PMIN)
                    sat_max = np.percentile(hsv[:,:,1], SAT_PMAX)
                    val_min = np.percentile(hsv[:,:,2], VAL_PMIN)
                    val_max = np.percentile(hsv[:,:,2], VAL_PMAX)
                    threshold_min = np.array([hue_min, sat_min, val_min], np.uint8)
                    threshold_max = np.array([hue_max, sat_max, val_max], np.uint8)
                    mask = cv2.inRange(hsv, threshold_min, threshold_max)            
                    mask = cv2.erode(mask, np.ones((3,3), np.uint8), iterations = 2)
                    mask = cv2.dilate(mask, np.ones((5,5), np.uint8), iterations = 1)
                    column_sum = mask.sum(axis=0) # vertical summation            
                    centroid = int(np.argmax(column_sum))
                    self.offset = centroid - PIXEL_WIDTH / 2.0
                    bgr[:,320,:] = 255
                    data_as_str = [str(i) for i in [time.time(), self.angle, self.encoder, self.offset, '\n']]
                    newline = ','.join(data_as_str)
                    csvfile.write(newline)
                    cv2.imshow('', bgr)
                    if cv2.waitKey(5) == 27:
                        pass
            except Exception as error:
                self.run_threads = False
                break                        
        csvfile.close()
        cv2.destroyAllWindows()
app = Trial()
app.run()
