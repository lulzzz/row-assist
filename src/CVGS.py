"""
V6 - Vision Speed Inference Extension

Runs as a ZMQ client.

The main function, run(), takes into account that the algorithm is slower than the 
camera framerate.
"""

__author__ = 'Trevor Stanhope'
__version__ = '1.0-alpha'

import scipy.cluster.hierarchy as hcluster
import pango
import shutil
import cv2, cv
import numpy as np
import time
import sys
from matplotlib import pyplot as plt
from itertools import cycle
import zmq
import json
from datetime import datetime
import gps as gpsd
import os
import thread
import serial
import pygtk
pygtk.require('2.0')
import gtk

class CVGS:
    """
    The Computer-vision Ground Speed Class
    Optional Arguments:
        capture [cv2.VideoCapture] :
        fov [rad] : Horizonal field-of-view,
        f [mm] : focal length
        d [mm] : depth from surface (default = 1000)
        roll [rad]: roll angle (default = 0)
        pitch [rad]: pitch angle (default = 0)
        yaw [rad]: yaw angle (default = 0)
        hessian [n/a]: iterations of hessian filter
        w [px] : image width
        h [px] : image height
    """
    def __init__(self, 
        capture=0,
        fov=0.75,
        f=6,
        aspect=1.33,
        d=1000,
        roll=0,
        pitch=0,
        yaw=0,
        hessian=300,
        w=640,
        h=480,
        neighbors=1,
        factor=0.7,
        gps=True,
        gps_device="/dev/ttyS0",
        gps_baud=38400,
        date_format="%Y-%m-%d %H:%M:%S",
        log_path='logs',
        num_fps_hist=3,
        fps=None
    ):
        # Shared variables
        self.log_name = ''
        self.lon = 0
        self.lat = 0
        self.speed = 0
        self.alt = 0
        self.num_matches = 0
        self.dt = [0] * num_fps_hist 
        self.bgr1 = np.zeros((640, 480, 3), np.uint8)
        self.bgr2 = np.zeros((640, 480, 3), np.uint8)
        
        # Things which can be changed at any time
        try:
            self.set_matchfactor(factor)
            self.set_fov(fov) # set the field of view (horizontal)
            self.set_aspect(aspect)
            self.set_focal_length(f)
            self.set_pitch(pitch) # 0 rad
            self.set_roll(roll) # 0 rad
            self.set_yaw(yaw) # 0 rad
            self.set_depth(d) # camera distance at center
            self.set_neighbors(neighbors)
            self.set_matcher(hessian)
        except Exception as e:
            print str(e)
        
    ## Set Matcher
    def set_matcher(self, hessian):
        """
        Set the keypoint matcher configuration, supports BF or FLANN
        """
        try:
            self.keypoint_filter = cv2.SURF(hessian, nOctaves=4, nOctaveLayers=2, extended=1, upright=1) 
            if self.neighbors == 1:
                self.matcher = cv2.BFMatcher(crossCheck=True)
            else:
                raise Exception("Only 1-neighbor cross-check matching is supported!")
        except Exception as e:
            raise Exception("Failed to generate a matcher")
    
    ## Close
    def close(self):
        """
        Close
        """ 
        try:
            pass
        except:
            raise Exception("Failed to close properly")
    
    ## Set Match Factor
    def set_matchfactor(self, factor):
        """
        Set Match Factor
        """
        if factor < 0:
            raise Exception("Cannot have match less than 1")
        else:
            self.factor = factor
                        
    ## Set Neighbors
    def set_neighbors(self, neighbors):
        """
        Set Neighbors
        """
        if neighbors < 0:
            raise Exception("Cannot have neighbors less than 0")
        else:
            self.neighbors = neighbors                

    ## Set distance at center of frame
    def set_depth(self, d):
        """
        Set distance at center of frame
        d : depth of view [m]
        """
        if d <= 0:
            raise Exception("Improper distance")
        else:
            self.d = d
            
    ## Set Pitch
    def set_pitch(self, pitch):
        """
        pitch [rad]
        0 is orthogonal to surface 
        """
        if pitch < 0:
            raise Exception("Cannot have negative inclination")
        elif pitch > np.pi/2.0:
            raise Exception("Cannot have inclination parallel to surface")
        else:
            self.pitch = pitch
            
    ## Set Roll
    def set_roll(self, roll):
        """
        Set Roll
        roll [rad]
        0 is parallel to surface 
        """
        self.roll = roll
        
    ## Set Yaw (relative to direction of travel)
    def set_yaw(self, yaw):
        """
        Set Yaw (relative to direction of travel)
        yaw [rad]
        0 is parallel to direction of travel 
        """
        self.yaw = yaw
    
    ## Set Field-of-View
    def set_fov(self, fov):
        """
        Set Field-of-View
        fov [rad]
        """
        if fov <= 0:
            raise Exception("Cannot have negative FOV")
        if fov >= np.pi:
            raise Exception("Cannot have FOV greater than Pi radians")
        else:
            self.fov = fov

    ## Set Aspect Ratio
    def set_aspect(self, aspect):
        """
        Set Aspect Ratio
        aspect [constant]
        """
        if aspect <= 0:
            raise Exception("Cannot have negative aspect ratio")
        else:
            self.aspect = aspect

    ## Set focal Length
    def set_focal_length(self, f):
        """
        Set Focal Length
        f [mm]
        """
        if f <= 0:
            raise Exception("Cannot have negative focal length")
        else:
            self.f = f
                
    ## Match Images
    def match_images(self, bgr1, bgr2):
        """
        Match Images
        Find (good) pairs of matching points between two images
        Returns: [ (pt1, pt2), ... ]
        """
        try:
            gray1 = cv2.cvtColor(bgr1, cv2.COLOR_BGR2GRAY)
            gray2 = cv2.cvtColor(bgr2, cv2.COLOR_BGR2GRAY)
            (self.w, self.h) = gray1.shape
            (pts1, desc1) = self.keypoint_filter.detectAndCompute(gray1, None)
            (pts2, desc2) = self.keypoint_filter.detectAndCompute(gray2, None)
            matching_pairs = []
            if pts1 and pts2:
                all_matches = self.matcher.knnMatch(desc1, desc2, k=self.neighbors)
                for m in all_matches:
                    if len(m) > 0:
                        pt1 = pts1[m[0].queryIdx]
                        pt2 = pts2[m[0].trainIdx]
                        xy1 = (pt1.pt[0], pt1.pt[1])
                        xy2 = (pt2.pt[0], pt2.pt[1])
                        matching_pairs.append((xy1, xy2))
                return matching_pairs
            else:
                raise Exception('No points to match!')
        except Exception as e:
            raise e
        
    # Distance
    def vector(self, pt1, pt2, project=False):
        """
        Distance between two keypoints, where keypoints are in units of pixels
        Arguments: pt1 : (int x1, int y1), pt2 : (int x2, int y2)
        Returns: distance : float
        """
        (x1, y1) = pt1
        (x2, y2) = pt2
        if project:
            (x1, y1) = self.project(x1, y1)
            (x2, y2) = self.project(x2, y2)
        dist = np.sqrt( (x2 - x1)**2 + (y2 - y1)**2 )
        theta = np.arctan2(float(y2 - y1), float(x2 - x1))
        return dist, theta
        
    ## Rectilinear Projection
    def project(self, x, y):
        """
        Project points from pixels to real units
        Required arguments:
            x : 
            y : 
        Optional arguments:
            h : height of camera from surface (any units)
            fov : horizontal field of view
            w : width of image (in pixels)
            pitch : angle of inclination (in radians)
            f : focal length
        Returns:
            (X, Y): point location
        """
        f = (2.0 / self.aspect) * np.tan(self.fov / (2.0))
        l = self.w / f
        theta = np.arctan(y / l)
        Y = self.d / np.tan( (np.pi / 2.0 - self.pitch) - theta)
        X = x * np.sqrt( (self.d**2 + Y**2) / (l**2 + y**2) )
        return (X, Y)
    
    ## Estimate Velocity
    def get_velocity(self, bgr1, bgr2, fps, output_units="kilometers", dilation = 2.5):
        """
        Optional:
            dt : time differential between bgr1 and bgr2
            p_min : minimum percentile for change in distance
            p_max : maximum percentile for change in distance
            output_units: either 'mph', 
        Returns:
            v : the estimated speed of travel
            t : the estimated angle moved between two keypoints
            pairs : matching pairs between bgr1 and bgr2
            bgr1 : the first image
            bgr2 : the second image
            
        """
        # Convert to time differential form
        try:
            dt = 1/float(fps) * dilation
        except:
            raise Exception("Failed to convert to seconds")
            
        # Match keypoint pairs
        try:
            pairs = self.match_images(bgr1, bgr2)
        except Exception as e:
            raise e

        # Convert units
        try:
            vectors = [self.vector(pt1, pt2, project=True) for (pt1, pt2) in pairs]
            [dists, thetas] = zip(*vectors)
            dists = np.array(dists)
            thetas = np.array(thetas)
            if output_units=="kilometers":
                v_all=(3.6 / 1000.0) * (dists / dt) # convert from m/s to km/hr
            elif output_units=="miles":
                v_all=(2.2369356 / 1000.0) * (dists / dt) # convert from m/s to miles/hr
            elif output_units=="meters":
                v_all=(0.001) * (dists / dt)
            else:
                v_all = (dists / dt)
            t_all = thetas * 180 / np.pi
        except Exception as e:
            raise e
        
        # Filter for best
        t_rounded = np.abs(np.around(t_all, 0).astype(np.int32))
        t_counts = np.bincount(t_rounded)
        t_mode = np.argmax(t_counts)
        t_best = np.isclose(t_rounded, t_mode, atol=1)
        v_best = np.mean(v_all[t_best])
        return v_best
            
if __name__ == '__main__':
    test = CVGS()
