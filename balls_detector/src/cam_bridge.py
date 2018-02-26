#!/usr/bin/env python
'''
We get a raw image from the connected camera, webcam or kinect, and broadcast
its pixels arra on a topic called /cam/image/rgb for onward processing by
other nodes

Authors:
    1.  Sleiman Safaoui
    2.  Kaveh Fathian
    3.  Olalekan Ogunmolu

July 14, 2017
########################## KINECT NOT TESTED YET ########################
'''
from __future__ import print_function

#sys files import
import cv2
import os
import argparse

#ros imports
import rospy
import roslib
roslib.load_manifest('balls_detector')
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


parser = argparse.ArgumentParser(description='Image Source')
parser.add_argument("--img_src", type=str, default="w")
input_arg = parser.parse_args(rospy.myargv()[1:])

class ImagePublisher:

    def __init__(self, input_arg):
        self.bridge = CvBridge()
        self.keep_running = True
        self.cam_pub = rospy.Publisher("/cam/image/rgb", Image, queue_size=10)

    def get_rgb(self, input_arg, *args):
        if input_arg.img_src == "w":
            ret, self.cam_image = args[0].read()
        elif input_arg.img_src == "k": #TODO check if kinect works
            self.cam_image = data[:, :, ::-1] #RGB -> BGR
        else:
            return

        try:
            self.cam_pub.publish(self.bridge.cv2_to_imgmsg(self.cam_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def body(self, input_arg, *args):
        if input_arg.img_src == "k":
            if not self.keep_running:
                raise freenect.Kill
        else:
            return

def main(input_arg):
    if not (input_arg.img_src == "w" or input_arg.img_src == "k"):
        rospy.loginfo("You can only use kinect or a webcam")
        os._exit()
    if input_arg.img_src == "w":
	# TODO(gazialankus) make this a parameter
        cam = cv2.VideoCapture("rtsp://192.168.137.17:8554/1.3gp") # change int to corresponding camera number
        # cam = cv2.VideoCapture(0) # change int to corresponding camera number
    else:
        import freenect

    rospy.init_node("cam_rgb_pub")
    ic = ImagePublisher(input_arg)
    rate = rospy.Rate(30)
    try:
        while not rospy.is_shutdown():
            if input_arg.img_src == "w":
                ic.get_rgb(input_arg, cam)
            else:
                freenect.runloop(video=ic.get_rgb(input_arg), body=ic.body(input_arg))
            rate.sleep()
    except KeyboardInterrupt:
        print("shutting down ros")

if __name__ == '__main__':
    try:
        main(input_arg)
    except rospy.ROSInterruptException:
        pass
