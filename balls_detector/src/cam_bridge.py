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
parser.add_argument("--img_src", type=str, default="k")
parser.add_argument("--cam_address", type=str, default=0)
input_arg = parser.parse_args(rospy.myargv()[1:])

class ImagePublisher:

    def __init__(self):
        self.bridge = CvBridge()
        self.keep_running = True
        self.cam_pub = rospy.Publisher("/cam/image/rgb", Image, queue_size=10)

    def get_rgb(self, *args):
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

    def body(self, *args):
        if input_arg.img_src == "k":
            if not self.keep_running:
                raise freenect.Kill
        else:
            return

def main():
    rospy.init_node("cam_rgb_pub")

    input_arg.img_src = rospy.get_param("~img_src", input_arg.img_src)
    input_arg.cam_address = rospy.get_param("~cam_address", input_arg.cam_address)

    if not (input_arg.img_src == "w" or input_arg.img_src == "k"):
        rospy.loginfo("You can only use kinect or a webcam")
        os._exit()
    if input_arg.img_src == "w":
        cam = cv2.VideoCapture(0)
        cam.set(3, 960)
        cam.set(4, 720)
    else:
        import freenect


    ic = ImagePublisher()
    rate = rospy.Rate(30)
    try:
        while not rospy.is_shutdown():
            if input_arg.img_src == "w":
                ic.get_rgb(cam)
            else:
                freenect.runloop(video=ic.get_rgb(), body=ic.body())
            rate.sleep()
    except KeyboardInterrupt:
        print("shutting down ros")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
