# Row Assist
Control system for assisting vehicle guidance during row-crop field operations

## Installation
* Requires SciPy 0.11 or higher
* Requires OpenCV 2.4.6 or higher
* Requires NumPy 1.8 or higher
* Requires Python 2.7.6 __(NOT Python 3.x.x)__

To install the system, simply run the install script:
    
    sh install.sh
    
## GPS
sudo gpsd /dev/ttyS0

## VideoCapture
sudo rmmod uvcvideo
sudo modprobe uvcvideo nodrop=1 timeout=5000 quirks=0x80

##
Encoder: 1275 steps / Rev of Wheel
Gearbox Ratio: 4.25:1
Stepper Motor: 850 steps / Rev of Wheel

