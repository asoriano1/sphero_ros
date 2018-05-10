#!/usr/bin/env python2

'''
We modify the camera parameters, in particular we set exposure to manual mode,
we reduce exposure to a minimum value, and we turn off autofocus

Reducing exposure allows for the elimination of the objects that are not bright
and allows for better detection of the robots
Turning off autofocus reduces distortion in image and detection that arise from
the camera trying to focus continuously

Authors:
    1.  Sleiman Safaoui
    2.  Kaveh Fathian
    3.  Olalekan Ogunmolu

July 11, 2017
'''

import os

# change video1 into video<X> where X is the derired webcam
os.system('v4l2-ctl -d /dev/video0 -c exposure_auto=1')

# reduce exposure to min value
os.system('v4l2-ctl -d /dev/video0 -c exposure_absolute=3')

# turn off autofocus
os.system('v4l2-ctl -d /dev/video0 -c focus_auto=0')

#TODO: extend support to Kinect
