#!/usr/bin/env python

__author__ = 'Trevor Stanhope'
__version__ = '0.1'
__date__ = 'Nov 4 2014'

# Imports
from ctypes import *
import sys
from time import sleep
from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException
from Phidgets.Events.Events import AttachEventArgs, DetachEventArgs, ErrorEventArgs, InputChangeEventArgs, CurrentChangeEventArgs, StepperPositionChangeEventArgs, VelocityChangeEventArgs
from Phidgets.Devices.Stepper import Stepper

# Create a stepper object
try:
    stepper = Stepper()
except RuntimeError as e:
    print("Runtime Exception: %s" % e.details)
    print("Exiting....")
    exit(1)

# Information Display Function
def DisplayDeviceInfo():
    print("%8s, %30s, %10d, %8d" % (stepper.isAttached(), stepper.getDeviceName(), stepper.getSerialNum(), stepper.getDeviceVersion()))
    print("Number of Motors: %i" % (stepper.getMotorCount()))

# Event Handler Callback Functions
def StepperAttached(e):
    attached = e.device
    print("Stepper %i Attached!" % (attached.getSerialNum()))

def StepperDetached(e):
    detached = e.device
    print("Stepper %i Detached!" % (detached.getSerialNum()))

def StepperError(e):
    try:
        source = e.device
        print("Stepper %i: Phidget Error %i: %s" % (source.getSerialNum(), e.eCode, e.description))
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))

def StepperCurrentChanged(e):
    source = e.device
    # print("Stepper %i: Motor %i -- Current Draw: %6f" % (source.getSerialNum(), e.index, e.current))

def StepperInputChanged(e):
    source = e.device
    # print("Stepper %i: Input %i -- State: %s" % (source.getSerialNum(), e.index, e.state))

def StepperPositionChanged(e):
    source = e.device
    # print("Stepper %i: Motor %i -- Position: %f" % (source.getSerialNum(), e.index, e.position))

def StepperVelocityChanged(e):
    source = e.device
    print("Stepper %i: Motor %i -- Velocity: %f" % (source.getSerialNum(), e.index, e.velocity))

# Main Program Code
try:
    stepper.setOnAttachHandler(StepperAttached)
    stepper.setOnDetachHandler(StepperDetached)
    stepper.setOnErrorhandler(StepperError)
    stepper.setOnCurrentChangeHandler(StepperCurrentChanged)
    stepper.setOnInputChangeHandler(StepperInputChanged)
    stepper.setOnPositionChangeHandler(StepperPositionChanged)
    stepper.setOnVelocityChangeHandler(StepperVelocityChanged)
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    print("Exiting....")
    exit(1)

# Initialize Stepper
print("Opening phidget object....")
try:
    stepper.openPhidget()
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    print("Exiting....")
    exit(1)

# Attach stepper
print("Waiting for attach....")
try:
    stepper.waitForAttach(10000)
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    try:
        stepper.closePhidget()
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Exiting....")
        exit(1)
    print("Exiting....")
    exit(1)
else:
    DisplayDeviceInfo()

try:
    sleep(1)
    p = int(raw_input("Set current position: "))
    stepper.setCurrentPosition(0, p)
    
    while True:
        try:
            print("Disengaging the motor ...")
            stepper.setEngaged(0, False)
            vel = int(raw_input("Specify Velocity: "))
            stepper.setVelocityLimit(0, vel)
            acc = int(raw_input("Specify Acceleration: "))
            stepper.setAcceleration(0, acc)
            amp = float(raw_input("Specify Amperage: "))
            stepper.setCurrentLimit(0, amp)
            print("Engaging the motor ...")
            stepper.setEngaged(0, True)
            sleep(2)
            while True:
                try:
                    q = int(raw_input("Set target position: "))
                    stepper.setTargetPosition(0, q)
                    while stepper.getCurrentPosition(0) != q:
                        pass
                    sleep(2)
                except KeyboardInterrupt:
                    break
        except Exception as e:
            print str(e)
            break
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    print("Exiting....")
    exit(1)
except KeyboardInterrupt:
    print("Closing...")
    try:
        stepper.setEngaged(0, False)
        sleep(1)
        stepper.closePhidget()
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Exiting....")
        exit(1)

    print("Done.")
    exit(0)
