#!/usr/bin/env python2
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import threading

last_tagged_data = []
last_detected_data = []
last_ordered_data = []
last_crap_data = []
named_positions = {}

height = 0
width = 0
image_subscription = None
lock = threading.Lock()
image = None


def tagged_callback(data):
    rospy.loginfo(rospy.get_caller_id() + " tagged_callback %s", data.data)
    global last_tagged_data, isImageDirty

    last_tagged_data = data.data
    isImageDirty = True


def detected_callback(data):
    rospy.loginfo(rospy.get_caller_id() + " detected_callback %s", data.data)
    global last_detected_data, isImageDirty

    last_detected_data = data.data
    isImageDirty = True


def ordered_callback(data):
    rospy.loginfo(rospy.get_caller_id() + " ordered_callback %s", data.data)
    global last_ordered_data, isImageDirty

    last_ordered_data = data.data
    isImageDirty = True


def crap_callback(data):
    rospy.loginfo(rospy.get_caller_id() + " crap_callback %s", data.data)
    global last_crap_data, isImageDirty

    last_crap_data = data.data
    isImageDirty = True


def name_callback(data):
    rospy.loginfo(rospy.get_caller_id() + " name_callback %s", data.data)
    print(data._connection_header)
    name = data._connection_header['topic'].split('/')[1]
    global named_positions, isImageDirty

    named_positions[name] = data.data
    isImageDirty = True

# def update_window(event):
#     global height, width, image, isImageDirty
#
#     rospy.loginfo("update window called")
#
#     blank_image = np.zeros((1000, 1000, 3), np.uint8)
#     rospy.loginfo("1")
#     cv2.rectangle(blank_image, (200, 200), (400, 400), (255, 255, 255))
#     rospy.loginfo("11")
#
#     image = blank_image
#     isImageDirty = True
#
#     #cv2.imshow("Detected", blank_image)
#     rospy.loginfo("2")
#
#     rospy.loginfo("cam_overlay updated ***********************************")

isImageDirty = False

bridge = CvBridge()


def image_callback(event):
    global height, width, image_subscription, image, isImageDirty, bridge, lock

    height = event.height
    width = event.width

    # rospy.loginfo("***************height %s, width %s " % (height, width))

    isImageDirty = True

    with lock:
        image = bridge.imgmsg_to_cv2(event)


def draw_x(img, p, m, text_pos, text, line_color, text_color):
    cv2.line(img, (p[0] - m, p[1] - m), (p[0] + m, p[1] + m), line_color, 2)
    cv2.line(img, (p[0] - m, p[1] + m), (p[0] + m, p[1] - m), line_color, 2)

    cv2.putText(img, text, tuple(np.add(text_pos, [2, 2])), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1)
    cv2.putText(img, text, tuple(np.add(text_pos, [1, 1])), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 1)
    cv2.putText(img, text, tuple(text_pos), cv2.FONT_HERSHEY_PLAIN, 1, text_color, 1)


def draw_points_in_array(img, data, line_color, text_color, note, text_placement):
    m = 10
    if len(data) > 0:
        for i in range(0, len(data) / 2):
            p = (int(data[2 * i]), int(data[2 * i + 1]))

            v = i * 8 - 10

            if text_placement == "Right":
                text_pos = np.add(p, [2 * m, v])
            elif text_placement == "Left":
                text_pos = np.add(p, [-5 * m, v])
            elif text_placement == "Top":
                text_pos = np.add(p, [v, -2 * m])
            else:  # bottom
                text_pos = np.add(p, [v, 2 * m])

            draw_x(img, p, 10, text_pos, "%s%s" % (note, i), line_color, text_color)


def collect_sphero_names():
    global sphero_names

    all_topics_and_types = rospy.get_published_topics()
    names_set = set()
    for topicAndType in all_topics_and_types:
        topic = topicAndType[0]
        if topic.startswith('/sphero'):
            sphero_name = topic.split('/')[1]
            names_set.add(sphero_name)

    sphero_names = list(names_set)

def cam_overlay():
    global image_subscription, lock, image, sphero_names
    rospy.init_node('cam_overlay')

    rospy.loginfo("cam_overlay started !!!!!!!!!!!!!")

    collect_sphero_names()

    rospy.Subscriber("/locs/tagged", Float64MultiArray, tagged_callback)
    rospy.Subscriber("/locs/detected", Float64MultiArray, detected_callback)
    rospy.Subscriber("/locs/ordered", Float64MultiArray, ordered_callback)
    rospy.Subscriber("/locs/crap", Float64MultiArray, crap_callback)

    for name in sphero_names:
        topic = "/{0}/cam_image_pos".format(name)
        rospy.Subscriber(topic, Float64MultiArray, name_callback)

    image_subscription = rospy.Subscriber("/cam/image/rgb", Image, image_callback)

    cv2.namedWindow("Detected")
    cv2.moveWindow("Detected", 100, 500)
    #cv2.resizeWindow("Detected", 640, 480)

    while True:
        if image is not None and isImageDirty:
            #image = np.zeros((height, width, 3), np.uint8)


            # rospy.loginfo("============= BEFORE SHOW LOCK ======================")
            with lock:
                line_color = (0, 255, 0)
                crap_color = (255, 0, 0)
                text_color = (0, 255, 0)
                # write on the corner of screen if array empty
                draw_points_in_array(image, last_ordered_data, line_color, text_color, "Or", "Top")
                draw_points_in_array(image, last_detected_data, line_color, (0, 255, 255), "Dt", "Right")
                draw_points_in_array(image, last_tagged_data, line_color, (255, 0, 255), "Tg", "Left")
                draw_points_in_array(image, last_crap_data, crap_color, crap_color, "Cr", "Left")

                robotpositions = []

                for name in sphero_names:
                    if name in named_positions:
                        robotpositions += named_positions[name]

                robot_color = (0, 0, 255)
                draw_points_in_array(image, robotpositions, robot_color, robot_color, "R", "Top")


                # cv2.rectangle(image, (x, y), (x + 100, y + 100), (255, 255, 255))
                # x += 1
                # y += 1
                cv2.imshow("Detected", image)
            # rospy.loginfo("============= AFTER SHOW LOCK ======================")

        cv2.waitKey(20)

    #     blank_image = np.zeros((1000, 1000, 3), np.uint8)
    #     cv2.rectangle(blank_image, (x, y), (x + 100, y + 100), (255, 255, 255))
    #     x += 1
    #     y += 1
    #     cv2.imshow("Detected", blank_image)
    #
    #     while True:
    #
    #         cv2.waitKey(20)



#it won't work like this... maybe open a new window instead? or draw when updating the image. yes.
# how about another window? that way it's going to be clean, too...
# ok then. another window, on black. also set the refresh rate somehow.

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    cam_overlay()
