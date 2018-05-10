#!/usr/bin/env python2

'''
We modify the camera parameters, in particular we set exposure to auto mode,
and we turn off autofocus

Authors:
    1.  Sleiman Safaoui
    2.  Kaveh Fathian
    3.  Olalekan Ogunmolu

July 30, 2017
'''

import os

# change video1 into video<X> where X is the derired webcam
os.system('v4l2-ctl -d /dev/video1 -c exposure_auto=3')


# turn off autofocus
os.system('v4l2-ctl -d /dev/video1 -c focus_auto=0')

#TODO: extend support to Kinect
