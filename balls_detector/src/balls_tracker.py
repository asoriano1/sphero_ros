#!/usr/bin/env python
import rospy
import cv2
import numpy as np

from std_msgs.msg import String, Float64MultiArray, ColorRGBA
from munkres import Munkres


class PositionAndColor:
    def __init__(self, position, colorBGR):
        self.position = position
        self.colorBGR = colorBGR

    def __str__(self):
        return "p: {0} \tc: {1}".format(self.position, self.colorBGR)

    def __repr__(self):
        return "p: {0} \tc: {1}".format(self.position, self.colorBGR)


def cvbgr2sp(sp_color):
    return sp_color[2] / 255.0, sp_color[1] / 255.0, sp_color[0] / 255.0, 1.0


def sp2cvbgr(sp_color):
    return sp_color[2] * 255.0, sp_color[1] * 255.0, sp_color[0] * 255.0


def calculate_increase_between_hues(hue1, hue2):
    # hue is 0-180 circular.
    range = 180
    diff = hue2 - hue1

    while diff > range/2:
        diff -= range

    while diff < -range/2:
        diff += range

    return diff


def calculate_hue_increase(bgr1, bgr2):
    return calculate_increase_between_hues(bgr2hue(bgr1), bgr2hue(bgr2))


def bgr2hsv(color_bgr):
    b = np.zeros((1, 1, 3), np.uint8)
    b[0, 0] = color_bgr
    hsv = cv2.cvtColor(b, cv2.COLOR_BGR2HSV)
    return hsv


def bgr2hue(bgr):
    hsv = bgr2hsv(bgr)
    # [[[ 97  49 252]]]
    return hsv[0][0][0]


class BallsTracker:
    def __init__(self):
        pass

    hardcoded_blobs = [
        PositionAndColor(
            (694.4015502929688, 459.9547424316406),
            (234.45545796737767, 245.04642409033877, 193.02383939774154)
        ),
        PositionAndColor(
            (576.946044921875, 415.176513671875),
            (221.13832199546485, 198.6235827664399, 219.40136054421768)
        )
    ]

    current_blobs = []

    def detected_callback(self, msg):
        # TODO collect and save. do not mutate existing, others may grab old one fine.
        # would this stuff go into a message receiving function? yes I receive all together, great.

        num_blobs = len(msg.data) / 5

        new_blobs = []

        for i in range(0, num_blobs):
            p = i * 5
            blob = PositionAndColor((msg.data[p], msg.data[p + 1]), (msg.data[p + 2], msg.data[p + 3], msg.data[p + 4]))
            new_blobs.append(blob)

        self.current_blobs = new_blobs

            # TODO go through each entry and compare it to the desired colors as well as the previous positions

        detectedBlobColors = []  # get this from opencv
        detectedBlobRobotIndices = []  # use the matching above as well as history (kalman) to decide on this. if in doubt, initiate a tag ensuring protocol. -1s for non-robot.


        #cv2.cvtColor(CV_BGR2HSV)
        #
        #

        # first need to know which one is which. there can be static ones in the background.
        # have to play with colors because there can be better shiny candidates out there
        # so, all of them are white, then they get their own colors, then you see which one is which and which ones are bg.



        pass



    def all_spheros(self):
        return range(0, len(self.sphero_names))

    sp_white = (1, 1, 1, 1)
    sp_red = (1, 0, 0, 1)
    sp_green = (0, 1, 0, 1)

    sphero_names = None
    desired_robot_rgbs = [sp_red, sp_green]

    def set_color(self, sphero_indices, color):
        for i in sphero_indices:
            print("2")
            name = self.sphero_names[i]
            print("2")
            pub = rospy.Publisher("/{0}/set_color".format(name), ColorRGBA, latch=True, queue_size=1)
            print("2")
            pub.publish(*color)
            print("2")


    def collect_blobs(self):
        print("collect?")
        for i in range(0, len(self.current_blobs)):
            blob = self.current_blobs[i]
            print("pos: ")
            print(blob.position)
            print("col: ")
            print(blob.colorBGR)

        # create a copy maybe?
        return self.current_blobs


    def print_blobs(self, blobs):
        print("BLOBS:{0}".format(len(blobs)))

        for i in range(0, len(blobs)):
            self.print_blob(blobs[i])


    def print_blob(self, blob):
        print("blob pos: ")
        print(blob.position)
        print("bolb colorBGR: ")
        colorBGR = blob.colorBGR
        print(colorBGR)

        hsv = bgr2hsv(colorBGR)

        print("\nblob HSV: ")
        print(hsv)

    def extract_spheros_and_crap_blobs(self, white_blobs, colored_blobs, sphero_names):
        # step 1: match the points using hungarian

        # row: white_blobs
        # col: colored_blobs

        distance_mat = np.empty([len(white_blobs), len(colored_blobs)])

        for wi in range(0, len(white_blobs)):
            for ci in range(0, len(colored_blobs)):
                white_pos = white_blobs[wi].position
                colored_pos = colored_blobs[ci].position
                v = np.array(white_pos) - np.array(colored_pos)
                d = np.linalg.norm(v)
                distance_mat[wi, ci] = d

        munkres = Munkres()
        indices = munkres.compute(distance_mat) # this has the assignments. nothing else is necessary.

        # step 2: calculate the changes in hue
        #           the one that decreased the most is green
        #           the one that increased the most is red
        # all others from both are dead blobs, remember those locations
        # output:
        # for each named sphero
        #   pos/color
        # for each other dead blob
        #   pos/color

        first = True
        ci_least_increment = 0
        wi_least_increment = 0
        least_increment_value = 0
        ci_most_increment = 0
        wi_most_increment = 0
        most_increment_value = 0

        # calculateHueChange

        for wi, ci in indices:
            hue_increase = calculate_hue_increase(white_blobs[wi].colorBGR, colored_blobs[ci].colorBGR)

            if first or hue_increase < least_increment_value:
                ci_least_increment = ci
                wi_least_increment = wi
                least_increment_value = hue_increase

            if first or hue_increase > most_increment_value:
                ci_most_increment = ci
                wi_most_increment = wi
                most_increment_value = hue_increase

            first = False

        # this didn't work with my cellphone in the mix... TODO print everything out and understand what went down
        ci_red = ci_most_increment
        ci_green = ci_least_increment

        named_spheros = []
        crap_blobs = []

        for i in range(0, len(sphero_names)):
            selected = None

            if self.desired_robot_rgbs[i] == self.sp_red:
                selected = colored_blobs[ci_red]
                print("{0} is red".format(i))

            elif self.desired_robot_rgbs[i] == self.sp_green:
                selected = colored_blobs[ci_green]
                print("{0} is green".format(i))

            named_spheros.append(selected)

        # risking double-counting here to prevent having different blinking crap sources from becoming one.
        for i in range(0, len(colored_blobs)):
            if i != ci_least_increment and i != ci_most_increment:
                crap_blobs.append(colored_blobs[i])

        for i in range(0, len(white_blobs)):
            if i != wi_least_increment and i != wi_most_increment:
                crap_blobs.append(white_blobs[i])

        return named_spheros, crap_blobs


    def main(self):
        rospy.init_node('balls_tracker', anonymous=True)

        self.collect_sphero_names()

        print("1")
        num_rob = len(self.sphero_names)
        print("1")

        # now I know how many spheros are there
        # I also receive the detected positions

        cam_delay_secs = 5

        rospy.Subscriber("/locs/detected_with_color", Float64MultiArray, self.detected_callback)

        self.set_color(self.all_spheros(), self.sp_white)

        rospy.sleep(cam_delay_secs)

        rospy.loginfo("Collecting white blobs for each sphero.")
        white_blobs = self.collect_blobs()

        # would be better to use the same rois, but it's not done here so I won't. just a thought.
        rospy.loginfo("Setting colors and will collect them back from blobs.")

        for i in range(0, len(self.sphero_names)):
            self.set_color([i], self.desired_robot_rgbs[i])

        rospy.sleep(cam_delay_secs)

        rospy.loginfo("Collecting colored blobs for each sphero.")
        colored_blobs = self.collect_blobs()

        print("\n\nwhite_blobs\n")
        self.print_blobs(white_blobs)
        print("\n\ncolored_blobs\n")
        self.print_blobs(colored_blobs)

        # ok great, now let's convert them to hsv. looks good.

        # step 1: match the points using hungarian
        # step 2: calculate the changes in hue
        #           the one that decreased the most is green
        #           the one that increased the most is red
        # all others from both are dead blobs, remember those locations
        # output:
        # for each named sphero
        #   pos/color
        # for each other dead blob
        #   pos/color

        named_spheros, crap_blobs = self.extract_spheros_and_crap_blobs(white_blobs, colored_blobs, self.sphero_names)

        print("\n\nnamed: ")
        print(named_spheros)
        print("\n\ncrap: ")
        print(crap_blobs)






        # what if it flickers? should I keep detected ones for a couple of frames? no cuz then you'll have to match existing ones with prev frame.

        # so, I grab the current detected positions and act on it
        # do the tagging protocol first.
        # then publish/subscribe under the existing robots. one level down maybe, like /sphero_2342/cam/pos etc.


        # constant tagging:
        #   I have all the robots on
        #   decide on a color for each robot
        #   among the tagged points, choose the one that's closest to the color

        # ensured tagging and with multiple robots
        #   turn one off
        #   see which one turned off
        #   if multiple, retry and remember. intersection is our robot. the other goes into the ignore list.




        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def collect_sphero_names(self):
        all_topics_and_types = rospy.get_published_topics()
        names_set = set()
        for topicAndType in all_topics_and_types:
            topic = topicAndType[0]
            if topic.startswith('/sphero'):
                sphero_name = topic.split('/')[1]
                names_set.add(sphero_name)

        self.sphero_names = list(names_set
                                 )
        print("sphero names:")

        for name in self.sphero_names:
            print(name)
        print("^sphero names:")

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


if __name__ == '__main__':
    try:
        BallsTracker().main()

    except rospy.ROSInterruptException:
        pass
