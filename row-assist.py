"""
Row Assist
Precision Agriculture and Soil Sensing Group (PASS)
McGill University, Department of Bioresource Engineering

"""

__author__ = 'Trevor Stanhope'
__version__ = '1.0'

## Libraries
import cv2, cv
from src import CVGS
import serial
import json, ast
import numpy as np
import scipy.signal as sig
from matplotlib import pyplot as plt
import thread
import gps
import sys, os, time
from datetime import datetime
from ctypes import *
from time import sleep
from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException
from Phidgets.Events.Events import AttachEventArgs, DetachEventArgs, ErrorEventArgs, InputChangeEventArgs, CurrentChangeEventArgs, StepperPositionChangeEventArgs, VelocityChangeEventArgs
from Phidgets.Devices.Stepper import Stepper
from src.fpid import fpid

sys.settrace
## Constants
try:
    CONFIG_FILE = '%s' % sys.argv[1]
except Exception as err:
    CONFIG_FILE = "settings.json"

## Class
class RowAssist:

    def pretty_print(self, task, msg, *args):
        try:
            date = datetime.strftime(datetime.now(), "%H:%M:%S.%f")    
            output = "%s\t%s\t%s" % (date, task, msg)
            print output  
        except:
            pass

    def __init__(self, config_file):
        
        # Load Config
        self.pretty_print("CONFIG", "Loading %s" % config_file)
        self.config = json.loads(open(config_file).read())
        for key in self.config:
            try:
                getattr(self, key)
            except AttributeError as error:
                setattr(self, key, self.config[key])

        self.bgr1 = np.zeros((640, 480, 3), np.uint8)
        self.bgr2 = np.zeros((640, 480, 3), np.uint8)
        self.cv_speed = 0.0

        # Initializers
        self.run_threads = True
        self.init_log() # it's best to run the log first to catch all events
        self.init_cameras()
        self.init_cv_groundspeed()
        self.init_stepper()
        self.init_controller()
        self.init_pid()
        self.init_gps()
        self.init_qlearner()

    ## Initialize Display
    def init_display(self):
        thread.start_new_thread(self.update_display, ())
    
    ## Initialize Ground Speed Matcher
    def init_cv_groundspeed(self):
        """
        Initialize CV GroundSpeed
        """
        self.pretty_print('GSV6', 'Initializing Groundspeed Matcher ...')
        try:
            self.CVGS = CVGS.CVGS()
            thread.start_new_thread(self.update_groundspeed, ())  
        except Exception as e:
            raise e
    
    # Initialize QLearner
    def init_qlearner(self):
        """
        Initialize Q-Learner
        """
        self.pretty_print('QLRN', 'Initializing Q-Learner ...')
        try:
            """
            X: STEERING_WHEEL POSITION
            Y: ERROR
            """
            self.qmatrix = np.zeros((self.OUTPUT_MAX, self.CAMERA_WIDTH, 3), np.uint8)
        except Exception as e:
            raise e
                
    # Initialize Cameras
    def init_cameras(self):
        """
        Initialize Camera
        """
        # Setting variables
        self.pretty_print('CAM', 'Initializing CV Variables ...')
        if self.CAMERA_ROTATED:
            self.CAMERA_HEIGHT, self.CAMERA_WIDTH = self.CAMERA_WIDTH, self.CAMERA_HEIGHT # flip dimensions if rotated
        self.CAMERA_CENTER = self.CAMERA_WIDTH / 2
        self.pretty_print('CAM', 'Camera Width: %d px' % self.CAMERA_WIDTH)
        self.pretty_print('CAM', 'Camera Height: %d px' % self.CAMERA_HEIGHT)
        self.pretty_print('CAM', 'Camera Center: %d px' % self.CAMERA_CENTER)
        self.pretty_print('CAM', 'Camera Depth: %d cm' % self.CAMERA_DEPTH)
        self.pretty_print('CAM', 'Camera FOV: %f rad' % self.CAMERA_FOV)
        self.pretty_print('INIT', 'Image Center: %d px' % self.CAMERA_CENTER)
        self.GROUND_WIDTH = 2 * self.CAMERA_DEPTH * np.tan(self.CAMERA_FOV / 2.0)
        self.pretty_print('CAM', 'Ground Width: %d cm' % self.GROUND_WIDTH)
        self.pretty_print('CAM', 'Error Tolerance: +/- %d cm' % self.ERROR_TOLERANCE)
        self.PIXEL_PER_CM = self.CAMERA_WIDTH / self.GROUND_WIDTH
        self.pretty_print('CAM', 'Pixel-per-cm: %d px/cm' % self.PIXEL_PER_CM)
        self.PIXEL_RANGE = int(self.PIXEL_PER_CM * self.ERROR_TOLERANCE) 
        self.pretty_print('CAM', 'Pixel Range: +/- %d px' % self.PIXEL_RANGE)
        self.PIXEL_MIN = self.CAMERA_CENTER - self.PIXEL_RANGE
        self.PIXEL_MAX = self.CAMERA_CENTER + self.PIXEL_RANGE 
        
        # Set Thresholds     
        self.threshold_min = np.array([self.HUE_MIN, self.SAT_MIN, self.VAL_MIN], np.uint8)
        self.threshold_max = np.array([self.HUE_MAX, self.SAT_MAX, self.VAL_MAX], np.uint8)
        
        # Attempt to set each camera index/name
        self.pretty_print('CAM', 'Initializing Cameras')
        self.cameras = [None] * self.CAMERAS
        self.images = [None] * self.CAMERAS
        self.masks = [None] * self.CAMERAS
        for i in range(self.CAMERAS):
            try:
                self.pretty_print('CAM', 'Attaching Camera #%d' % i)
                cam = cv2.VideoCapture(i)
                cam.set(cv.CV_CAP_PROP_SATURATION, self.CAMERA_SATURATION)
                if not self.CAMERA_ROTATED:
                    cam.set(cv.CV_CAP_PROP_FRAME_WIDTH, self.CAMERA_WIDTH)
                    cam.set(cv.CV_CAP_PROP_FRAME_HEIGHT, self.CAMERA_HEIGHT)
                else:
                    cam.set(cv.CV_CAP_PROP_FRAME_WIDTH, self.CAMERA_HEIGHT)
                    cam.set(cv.CV_CAP_PROP_FRAME_HEIGHT, self.CAMERA_WIDTH)
                (s, bgr) = cam.read()
                self.images[i] = bgr
                self.cameras[i] = cam
                self.pretty_print('CAM', 'Camera #%d OK' % i)
            except Exception as error:
                self.pretty_print('CAM', 'ERROR: %s' % str(error))
    
    # Initialize PID Controller
    def init_pid(self):
        self.pretty_print('PID', 'Initializing Control System')
        # Initialize variables
        try:
            self.pretty_print('PID', 'Default number of samples: %d' % self.NUM_SAMPLES)
            self.offset_history = [0] * self.NUM_SAMPLES
            self.pretty_print('PID', 'Setup OK')
        except Exception as error:
            self.pretty_print('PID', 'ERROR: %s' % str(error))
        self.average = 0
        self.hist = [0] * 20
        self.estimated = 0
        self.output = 0

        # Generate control sys
        if self.ALGORITHM == 'PID':
            self.sys = fpid.PID(P=self.P_COEF, I=self.I_COEF, D=self.D_COEF)
        elif self.ALGORITHM == 'FUZZY':
            self.sys = fpid.FPID()
    
    # Initialize Log
    def init_log(self):
        self.pretty_print('LOG', 'Initializing Log')
        self.LOG_NAME = datetime.strftime(datetime.now(), self.LOG_FORMAT)
        self.pretty_print('LOG', 'New log file: %s' % self.LOG_NAME)
        gps_params = ['gps_lat', 'gps_long', 'gps_speed']
        nongps_params = ['time', 'cv_speed', 'est', 'avg', 'diff', 'output', 'steps','encoder','angle']
        if self.GPS_ON:    
            self.log_params = gps_params + nongps_params
        else:
            self.log_params = nongps_params
        try:
            self.log = open('logs/' + self.LOG_NAME + '.csv', 'w')
            self.log.write('OUTPUT_MIN ' + str(self.OUTPUT_MIN) + '\n')
            self.log.write('OUTPUT_MAX ' + str(self.OUTPUT_MAX) + '\n')
            self.log.write('P_COEF ' + str(self.P_COEF) + '\n')
            self.log.write('I_COEF ' + str(self.I_COEF) + '\n')
            self.log.write('D_COEF ' + str(self.D_COEF) + '\n')
            self.log.write('VELOCITY ' + str(self.VELOCITY) + '\n')
            self.log.write('ACCELERATION ' + str(self.ACCELERATION) + '\n')
            self.log.write(','.join(self.log_params + ['\n']))
            self.pretty_print('LOG', 'Setup OK')
            self.vid_writer = cv2.VideoWriter('logs/' + self.LOG_NAME + '.avi', cv.CV_FOURCC('M', 'J', 'P', 'G'), self.CAMERA_FPS, (self.CAMERA_WIDTH, self.CAMERA_HEIGHT), True)
        except Exception as error:
            raise error
    
    # Initialize Controller
    def init_controller(self):
        self.pretty_print('CTRL', 'Initializing controller ...')
        try:
            self.pretty_print('CTRL', 'Device: %s' % str(self.SERIAL_DEVICE))
            self.pretty_print('CTRL', 'Baud Rate: %s' % str(self.SERIAL_BAUD))
            self.controller = serial.Serial(self.SERIAL_DEVICE, self.SERIAL_BAUD, timeout=0.05)
            self.angle = 0
            self.angle_rate = 0
            self.encoder = 0
            self.encoder_rate = 0    
            self.encoder_rate_prev = 0
            thread.start_new_thread(self.update_controller, ())
            self.pretty_print('CTRL', 'Setup OK')
        except Exception as error:
            self.pretty_print('CTRL', 'ERROR: %s' % str(error))
            exit(1)
            
    # Initialize Stepper
    def init_stepper(self):

        # Constants
        self.pretty_print('STEP', 'Output Minimum: %d' % self.OUTPUT_MIN)
        self.pretty_print('STEP', 'Output Maximum: %d' % self.OUTPUT_MAX)

        # Create
        try:
            self.pretty_print("STEP", "Creating phidget object....")
            self.stepper = Stepper()
        except PhidgetException as e:
            self.pretty_print("STEP", "Phidget Exception %i: %s" % (e.code, e.details))

        # Open
        try:
            self.pretty_print("STEP", "Opening phidget object....")
            self.stepper.openPhidget()
        except PhidgetException as e:
            self.pretty_print("STEP", "Phidget Exception %i: %s" % (e.code, e.details))
        
        # Settings
        try:
            self.pretty_print("STEP", "Configuring stepper settings ...")
            self.stepper.setOnAttachHandler(self.StepperAttached)
            self.stepper.setOnDetachHandler(self.StepperDetached)
            self.stepper.setOnErrorhandler(self.StepperError)
            self.stepper.setOnCurrentChangeHandler(self.StepperCurrentChanged)
            self.stepper.setOnInputChangeHandler(self.StepperInputChanged)
            self.stepper.setOnPositionChangeHandler(self.StepperPositionChanged)
            self.stepper.setOnVelocityChangeHandler(self.StepperVelocityChanged)  
        except PhidgetException as e:
            self.pretty_print("STEP", "Phidget Exception %i: %s" % (e.code, e.details))

        # Attach
        try:
            self.pretty_print("STEP", "Attaching stepper motor ...")
            self.stepper.waitForAttach(1000)
        except PhidgetException as e:
            self.pretty_print("STEP", "Phidget Exception %i: %s" % (e.code, e.details))
            try:
                self.stepper.closePhidget()
            except PhidgetException as e:
                self.pretty_print("STEP", "Phidget Exception %i: %s" % (e.code, e.details))
            exit(1)
        else:
            self.DisplayDeviceInfo()
            
        # Engage
        try:
            self.pretty_print("STEP", "Engaging stepper motor ...")
            self.output = (self.OUTPUT_MIN + self.OUTPUT_MAX) / 2
            self.stepper.setCurrentPosition(0, (self.OUTPUT_MIN + self.OUTPUT_MAX) / 2)
            self.stepper.setTargetPosition(0, (self.OUTPUT_MIN + self.OUTPUT_MAX) / 2)
            self.stepper.setEngaged(0, False)
            self.stepper.setVelocityLimit(0, self.VELOCITY)
            self.stepper.setAcceleration(0, self.ACCELERATION)
            self.stepper.setCurrentLimit(0, self.AMPS)
            self.stepper.setEngaged(0, True)
        except Exception as error:
            self.pretty_print("STEP", "ERROR: %s" % str(error))
            exit(2)
            
    # Initialize GPS
    def init_gps(self):
        """ Initialize GPS """
        self.pretty_print('GPS', 'Initializing GPS ...')
        self.gps_latitude = 0
        self.gps_longitude = 0
        self.gps_altitude = 0
        self.gps_speed = 0
        try:
            self.gps = serial.Serial(self.GPS_DEVICE, self.GPS_BAUD)
            thread.start_new_thread(self.update_gps, ())
            self.pretty_print("GPS", "GPS connected")
        except Exception as err:
            self.pretty_print('GPS', 'WARNING: GPS not available! %s' % str(err))

    ## Update Learner
    def update_learner(self, ph1, e, group):
        self.qmatrix[ph1,e,:] = self.qmatrix[ph1,e,:] + group
        return group

    ## Capture Images
    def capture_images(self):
        """
        1. Attempt to capture an image
        2. Repeat for each capture interface
        """
        images = []
        for i in range(self.CAMERAS):
            try:
                (s, bgr) = self.cameras[i].read()
                if s and (self.images[i] is not None):
                    if self.CAMERA_ROTATED: bgr = cv2.transpose(bgr)
                    if np.all(bgr==self.images[i]):
                        images.append(None)
                        self.pretty_print('CAM', 'ERROR: Frozen frame')
                    else:
                        images.append(bgr)
                else:
                    self.pretty_print('CAM', 'ERROR: Capture failed')
                    self.cameras[i].release()
                    self.cameras[i] = cv2.VideoCapture(i)
                    (s, bgr) = self.cameras[i].read()
                    if s:
                        images.append(bgr)
                    else:
                        images.append(None)
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except:
                images.append(None)
        self.bgr2 = self.bgr1 
        self.bgr1 = images[0] # Update the BGR
        return images

    ## Plant Segmentation Filter
    def plant_filter(self, images):
        """
        1. RBG --> HSV
        2. Set minimum saturation equal to the mean saturation
        3. Set minimum value equal to the mean value
        4. Take hues within range from green-yellow to green-blue
        """
        masks = []
        for bgr in images:
            if bgr is not None:
                try:
                    bgr = np.rot90(bgr)
                    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
                    self.threshold_min[1] = np.percentile(hsv[:,:,1], 100 * self.SAT_MIN / 256) # overwrite the saturation minima
                    self.threshold_min[2] = np.percentile(hsv[:,:,2], 100 * self.VAL_MIN / 256) # overwrite the value minima
                    mask = cv2.inRange(hsv, self.threshold_min, self.threshold_max)
                    masks.append(mask)
                except KeyboardInterrupt:
                    raise KeyboardInterrupt
                except Exception as e:
                    raise e
                masks.append(None)
        return masks
        
    ## Find Plants
    def find_offset(self, masks):
        """
        1. Calculates the column summation of the mask
        2. Calculates the 95th percentile threshold of the column sum array
        3. Finds indicies which are greater than or equal to the threshold
        4. Finds the median of this array of indices
        5. Repeat for each mask
        """
        indices = []
        for mask in masks:
            if mask is not None:
                try:
                    column_sum = mask.sum(axis=0) # vertical summation            
                    centroid = int(np.argmax(column_sum) - self.CAMERA_CENTER + self.CAMERA_OFFSET)                   
                    indices.append(centroid)
                except KeyboardInterrupt:
                    raise KeyboardInterrupt
                except Exception as error:
                    self.pretty_print('OFF', '%s' % str(error))
            else:
                indices.append(self.CAMERA_OFFSET)
        return indices
        
    ## Best Guess for row based on multiple offsets from indices
    def estimate_error(self, indices):
        """
        Calculate errors for estimate, average, and differential
        """
        try:
            val = int(np.mean(indices))
            self.offset_history.append(val)
            while len(self.offset_history) > self.NUM_SAMPLES:
                self.offset_history.pop(0)
            sig.savitsky_golay(self.offset_history)[-1]
            e = int(self.offset_history[-1]) # get latest
            t = range(0, self.T_COEF) # the time frame in the past
            t_plus = range(self.T_COEF + 1, self.T_COEF *2) # the time frame in the future
            f = np.polyfit(t, self.offset_history[-self.T_COEF:], deg=self.REGRESSION_DEG)
            vals = np.polyval(f, t_plus)
            de = vals[-1] # differential error
            ie = int(np.mean(vals)) # integral error
        except Exception as error:
            self.pretty_print("ROW", "Error: %s" % str(error))
        return e, ie, de
                
    ## Calculate Output (Supports different algorithms)
    def calculate_output(self, e, ie, de, v, d_phi, d_phi_prev):
        """
        Calculates the PID output for the stepper
        Arguments: est, avg, diff, speed
        Requires: OUTPUT_MAX, OUTPUT_MIN, CENTER_OUTPUT
        Returns: output
        """
        p = e * self.P_COEF
        i = ie * self.I_COEF
        d = de * self.D_COEF
        output = self.sys.calculate(e, ie, de)
        return int(output)
        
    ## Read Controller
    def update_controller(self):
        """ Get info from controller """
        a = time.time()
        b = time.time()
        while self.run_threads:
            event = None
            try:
                s = self.controller.readline()
                event = json.loads(s)
                b = time.time() # end timer
                if event is not None:
                    self.encoder = event['a']
                    self.angle = event['b']
                    self.encoder_rate = (event['a'] - self.encoder) / (b - a)
                    self.encoder = event['a']
            except Exception as error:
                print str(error)
            a = time.time() # reset timer

    ## Set Stepper
    def set_stepper(self, vel):
        """ Set Stepper, returns the dead-reckoning number of steps/position """
        a = time.time()
        phi_estimated = self.stepper.getCurrentPosition(0)
        try:
            if self.stepper.isAttached():
                phi_target = phi_estimated + vel

                # Set Direction
                if vel < -self.DEADBAND:
                    output = self.OUTPUT_MIN
                elif vel > self.DEADBAND:                     
                    output = self.OUTPUT_MAX
                else:                     
                    output = phi_estimated
                self.stepper.setTargetPosition(0, output) # SETS THE OUTPUT

                # Set Velocity
                if abs(vel) > self.VELOCITY_MAX:
                    vel = self.VELOCITY_MAX
                self.stepper.setVelocityLimit(0, abs(vel))
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except Exception as e:
            self.close_stepper()
            raise e
        b = time.time()
        return phi_target

    ## Get Groundspeed
    def get_grounspeed(self, images):
        """ Get the current groundspeed """
        return self.cv_speed # Return current speed
    
    ## Log to Mongo
    def write_to_db(self, sample):
        """ Write results to the database """
        try:          
            assert self.collection is not None
            doc_id = self.collection.insert(sample)
            self.pretty_print('DB', 'Doc ID: %s' % str(doc_id))
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except Exception as e:
            raise Exception("Failed to insert document")
        return doc_id

    ## Write to Log
    def write_to_log(self, sample):
        """
        Write results to the log
        """
        a = time.time()
        try:          
            data = [str(sample[k]) for k in self.log_params]
            newline = ','.join(data + ['\n'])
            self.log.write(newline)
            self.vid_writer.write(self.images[0])
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except Exception as e:
            self.pretty_print("LOG", str(e))
            raise Exception("Failed to write to file document")
        b = time.time()
                    
    ## Update GPS
    def update_gps(self):  
        """
        1. Get the most recent GPS data
        2. Set global variables for lat, long and speed
        """
        while self.run_threads:
            try:
                sentence = self.gps.readline()
                sentence_parsed = sentence.rsplit(',')
                nmea_type = sentence_parsed[0]
                if nmea_type == '$GPVTG':
                    self.gps_speed = float(sentence_parsed[7])
                elif nmea_type == '$GPGGA':
                    self.gps_latitude = float(sentence_parsed[2])
                    self.gps_longitude = float(sentence_parsed[4])
                    self.gps_altitude = float(sentence_parsed[9])
            except Exception as e:
                self.gps_latitude = 0.0
                self.gps_longitude = 0.0
                self.gps_altitude = 0.0
                self.gps_speed = 0.0
    
    ## Estimate groundspeed (THREADED)
    def update_groundspeed(self, wait=0.05, hist_length=3):
        """ Needs: bgr1 and bgr2 """
        self.speed_hist = [0] * hist_length
        while self.run_threads:
            time.sleep(wait) # don't run too fast
            try:
                bgr1 = self.bgr1
                bgr2 = self.bgr2
            except Exception as e:
                raise e
            try:
                if not np.all(bgr1==bgr2):
                    cv_speed = self.CVGS.get_velocity(bgr1, bgr2, fps=self.CAMERA_FPS)
                    self.speed_hist.reverse()
                    self.speed_hist.pop()
                    self.speed_hist.reverse()
                    self.speed_hist.append(cv_speed)
                    self.cv_speed = np.mean(self.speed_hist)
            except Exception as error:
                self.pretty_print('CVGS', 'CV001: %s' % str(error))

    ## Update Display (THREADED)
    def update_display(self):
        """ Flash BGR capture to user """
        try:
            cv2.imshow('', self.images[0])
            if cv2.waitKey(5) == 0:
                pass
        except Exception as error:
            self.pretty_print('DISP', 'ERROR: %s' % str(error))
            
    ## Close Controller
    def close_stepper(self):
        """
        Close Controller
        """
        self.pretty_print('SYSTEM', 'Closing Stepper ...')
        try:
            self.stepper.setEngaged(0, False)
            self.stepper.closePhidget()
        except Exception as error:
            self.pretty_print('STEP', 'ERROR: %s' % str(error))
    
    ## Close
    def close(self):
        """
        Function to shutdown application safely
        1. Close windows
        2. Disable stepper
        3. Release capture interfaces 
        """
        self.pretty_print('SYSTEM', 'Shutting Down Now!')
        try:
            self.close_stepper() ## Disable stepper
        except Exception as error:
            self.pretty_print('STEP', 'ERROR: %s' % str(error))
        try:
            self.controller.close() ## Disable Arduino
        except Exception as error:
            self.pretty_print('ARD', 'ERROR: %s' % str(error))
        for i in range(len(self.cameras)):
            try:
                self.pretty_print('CAM', 'Closing Camera #%d ...' % i)
                self.cameras[i].release() ## Disable cameras
            except Exception as error:
                self.pretty_print('CAM', 'ERROR: %s' % str(error))
        cv2.destroyAllWindows() ## Close windows
        
    ## Run  
    def run(self):
        """
        Function for Run-time loop
        1. Get initial time
        2. Capture images
        3. Generate mask filter for plant matter
        4. Calculate indices of rows
        5. Estimate row from both images
        6. Get number of samples
        7. Calculate lateral error after filtering
        8. Send output response to stepper
        9. Throttle to desired frequency
        10. Log results to DB
        11. Display results
        """  
        while self.run_threads:
            try:
                a = time.time()
                images = self.capture_images()
                if not all(i is None for i in images):
                    cv_speed = self.get_grounspeed(images)
                    masks = self.plant_filter(images)
                    offsets = self.find_offset(masks)
                    (est, avg, diff) = self.estimate_error(offsets)
                    encoder = self.encoder
                    encoder_rate = self.encoder_rate
                    encoder_rate_prev = self.encoder_rate_prev
                    angle = self.angle
                    output = self.calculate_output(est, avg, diff, cv_speed, encoder_rate, encoder_rate_prev )
                    self.encoder_rate_prev = encoder_rate
                    steps = self.set_stepper(output)
                    sample = {
                        'offsets' : offsets, 
                        'est' : est,
                        'avg' : avg,
                        'diff' : diff,
                        'angle' : angle,
                        'encoder' : encoder,
                        'encoder_rate' : encoder_rate,
                        'output': output,
                        'steps' : steps,
                        'time' : datetime.strftime(datetime.now(), self.TIME_FORMAT),
                        'cv_speed' : cv_speed
                    }
                    if self.GPS_ON:
                        gps_sample = {
                            'gps_long' : self.gps_longitude,
                            'gps_lat' : self.gps_latitude,
                            'gps_alt' : self.gps_altitude,
                            'gps_speed' : self.gps_speed
                        }
                        sample.update(gps_sample)
                    if self.MONGO_ON:
                        doc_id = self.write_to_db(sample)
                    if self.LOGFILE_ON:
                        self.write_to_log(sample)
                    self.output = output
                    self.images = images
                    self.masks = masks
                    b = time.time()
                    if False : #images[0] is None:
                        cv2.imshow('', np.array(images[0], np.uint8).copy())
                        if cv2.waitKey(5) == 27:
                            pass
                    #self.update_display()
                    if self.VERBOSE:
                        self.pretty_print("STEP", "%d Hz\t%d stp\t%d mm\t%2.1f d\t%0.2f d/s" % ((1 / float(b-a)),
                                                                                                 int(output),
                                                                                                 est,
                                                                                                 encoder * 360 / float(self.ENCODER_RESOLUTION),
                                                                                                 encoder_rate * 360 / float(self.ENCODER_RESOLUTION)))
                else:
                    time.sleep(0.01)
            except KeyboardInterrupt as error:
                self.run_threads = False
                self.close()    
                break
            except UnboundLocalError as error:
                print "RUN " + str(error)
            except Exception as error:
                print "RUN " + str(error)

    # Information Display Function
    def DisplayDeviceInfo(self):
        self.pretty_print("STEP", "%8s, %30s, %10d, %8d" % (self.stepper.isAttached(), self.stepper.getDeviceName(), self.stepper.getSerialNum(), self.stepper.getDeviceVersion()))
        self.pretty_print("STEP", "Number of Motors: %i" % (self.stepper.getMotorCount()))

    # Event Handler Callback Functions
    def StepperAttached(self, e):
        attached = e.device
        self.pretty_print("STEP", "Stepper %i Attached!" % (attached.getSerialNum()))

    def StepperDetached(self, e):
        detached = e.device
        self.pretty_print("STEP", "Stepper %i Detached!" % (detached.getSerialNum()))

    def StepperError(self, e):
        try:
            source = e.device
            self.pretty_print("STEP", "Stepper %i: Phidget Error %i: %s" % (source.getSerialNum(), e.eCode, e.description))
        except PhidgetException as e:
            self.pretty_print("STEP", "Phidget Exception %i: %s" % (e.code, e.details))

    def StepperCurrentChanged(self, e):
        source = e.device
        #self.pretty_print("STEP", "Stepper %i: Motor %i -- Current Draw: %6f" % (source.getSerialNum(), e.index, e.current))

    def StepperInputChanged(self, e):
        source = e.device
        #self.pretty_print("STEP", "Stepper %i: Input %i -- State: %s" % (source.getSerialNum(), e.index, e.state))

    def StepperPositionChanged(self, e):
        source = e.device
        #self.pretty_print("STEP", "Stepper %i: Motor %i -- Position: %f" % (source.getSerialNum(), e.index, e.position))

    def StepperVelocityChanged(self, e):
        source = e.device
        #self.pretty_print("STEP", "Stepper %i: Motor %i -- Velocity: %f" % (source.getSerialNum(), e.index, e.velocity))
        
## Main
if __name__ == '__main__':
    session = RowAssist(CONFIG_FILE)
    session.run()
