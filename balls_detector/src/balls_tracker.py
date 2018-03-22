#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import Image


class BallsTracker:
    def __init__(self):
        pass

    def detected_callback(self, msg):
        # TODO collect and save. do not mutate existing, others may grab old one fine.
        # would this stuff go into a message receiving function? yes I receive all together, great.

        numBlobs = len(msg.data) / 5

        for i in range(0, numBlobs):
            # TODO go through each entry and compare it to the desired colors as well as the previous positions

        detectedBlobColors = []  # get this from opencv
        detectedBlobRobotIndices = []  # use the matching above as well as history (kalman) to decide on this. if in doubt, initiate a tag ensuring protocol. -1s for non-robot.




        pass


    sphero_names = None
    desired_robot_colors = [(1, 0, 0), (0, 1, 0)]

    def main(self):
        rospy.init_node('balls_tracker', anonymous=True)

        self.collect_sphero_names()

        num_rob = len(self.sphero_names)

        # now I know how many spheros are there
        # I also receive the detected positions

        rospy.Subscriber("/locs/detected_with_color", Float64MultiArray, self.detected_callback)
        rospy.Subscriber("/cam/image/rgb", Image, self.image_callback)


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
        self.sphero_names = set()
        for topicAndType in all_topics_and_types:
            topic = topicAndType[0]
            if topic.startswith('/sphero'):
                sphero_name = topic.split('/')[1]
                self.sphero_names.add(sphero_name)
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
