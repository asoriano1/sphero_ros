#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64MultiArray


class BallsTracker:
    def __init__(self):
        pass

    def detected_callback(self):
        pass

    def main(self):
        rospy.init_node('balls_tracker', anonymous=True)

        rospy.Subscriber("/locs/detected", Float64MultiArray, self.detected_callback)

        num_rob = rospy.get_param("~num_rob")

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


if __name__ == '__main__':
    try:
        BallsTracker().main()

    except rospy.ROSInterruptException:
        pass
