#!/usr/bin/env python2

from __future__ import print_function

"""
    OpenCV sphero_detector
    Implements Kaveh's Sphero detector algorithm
    April 27, 2017
"""
__author__ = 'Olalekan Ogunmolu <Olalekan.Ogunmolu@utdallas.edu>'
__version__ = '0.0'
__license__ = 'MIT'

import cv2
# import cv2
import time
import roslib
import rospy

import cv2
from distutils.version import LooseVersion
# if LooseVersion(cv2.__version__).version[0] == 2:
#     # Whatever OpenCV2 code
# else:
#     # Whatever OpenCV3 code
# roslib.load_manifest('sphero_run')

class Sphero_Detector(object):
    def __init__(self):

        self.windowName = "sphero_detection"

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.color = (0, 255, 0)

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, False)

        print("spinning")

    def decode_fourcc(self, v):
        v = int(v)
        return "".join([chr((v >> 8 * i) & 0xFF) for i in range(4)])

    def shutdown_hook(self):
        print("shutting down!")

    def getImage(self):
        cv2.namedWindow(self.windowName)

        convert_rgb = True
        fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        focus = int(min(self.cap.get(cv2.CAP_PROP_FOCUS) * 100, 2**31-1))  # ceil focus to C_LONG as Python3 int can go to +inf

        cv2.createTrackbar("FPS", "Video", fps, 30, lambda v: cap.set(cv2.CAP_PROP_FPS, v))
        cv2.createTrackbar("Focus", "Video", focus, 100, lambda v: cap.set(cv2.CAP_PROP_FOCUS, v / 100))

        status, img = self.cap.read()
        fourcc = self.decode_fourcc(self.cap.get(cv2.CAP_PROP_FOURCC))

        if not bool(self.cap.get(cv2.CAP_PROP_CONVERT_RGB)):
            if fourcc == "MJPG":
                img = cv2.imdecode(img, cv2.IMREAD_GRAYSCALE)
            elif fourcc == "YUYV":
                img = cv2.cvtColor(img, cv2.COLOR_YUV2GRAY_YUYV)
            else:
                print("unsupported format")
                # break

        cv2.putText(img, "Mode: {}".format(fourcc), (15, 40), self.font, 1.0, self.color)
        cv2.putText(img, "FPS: {}".format(fps), (15, 80), self.font, 1.0, self.color)
        cv2.imshow("Video", img)

        k = 0xFF & cv2.waitKey(1)

        if k == 27:
            rospy.on_shutdown(self.shutdown_hook)
        elif k == ord("g"):
            convert_rgb = not convert_rgb
            self.cap.set(cv2.CAP_PROP_CONVERT_RGB, convert_rgb)

def main():
    rospy.init_node("detector_node", anonymous=True)
    sd = Sphero_Detector()
    try:
        while not rospy.is_shutdown():
            sd.getImage()
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down ros")

if __name__ == '__main__':
    main()
