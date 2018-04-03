#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from pyquaternion import Quaternion


def radians_from_to_around(v1, v2, a):
    c = np.cross(v1, v2)

    d = np.dot(v1, v2)

    return np.arctan2(np.dot(c, a), d)


def rotate_around_with_radians(v1, a, radians):
    quat = Quaternion(axis=a, radians=radians)

    return quat.rotate(v1)


class BallsController:
    def __init__(self):
        self.sphero_names = None
        self.current_pos = {}
        self.goal_pos = {}
        self.rad_offset = {}

    def collect_sphero_names(self):
        all_topics_and_types = rospy.get_published_topics()
        names_set = set()
        for topicAndType in all_topics_and_types:
            topic = topicAndType[0]
            if topic.startswith('/sphero'):
                sphero_name = topic.split('/')[1]
                names_set.add(sphero_name)

        self.sphero_names = list(names_set)
        print("sphero names:")

        for name in self.sphero_names:
            print(name)
            print("{0}".format(name))
        print("^sphero names:")

    def listen_to_topics(self):

        for name in self.sphero_names:

            # desired position
            goal_pos_topic = "/{0}/cam_image_goal_pos".format(name)
            rospy.Subscriber(goal_pos_topic, Float64MultiArray, self.goal_pos_detected)

            # current position
            current_pos_topic = "/{0}/cam_image_pos".format(name)
            rospy.Subscriber(current_pos_topic, Float64MultiArray, self.current_pos_detected)

        # if they are different, action

    def goal_pos_detected(self, msg):
        print("goal pos detected")
        name = msg._connection_header['topic'].split('/')[1]

        print("name: {0}, pos: {1}".format(name, msg.data))
        self.goal_pos[name] = list(msg.data) + [0]  # copy

    def current_pos_detected(self, msg):
        name = msg._connection_header['topic'].split('/')[1]

        self.current_pos[name] = list(msg.data) + [0]  # copy

    def move_towards_goals(self):
        for name in self.sphero_names:
            self.rad_offset[name] = 0

        rate_fps = 10
        rate = rospy.Rate(rate_fps)
        speed = 20

        while not rospy.is_shutdown():
            # names that are both in goal_pos and current_pos
            name_set = set(self.goal_pos.keys()).intersection(set(self.current_pos.keys()))

            for name in name_set:
                print("\nWILL MOVE {0}".format(name))
                pos = self.current_pos[name]
                goal = self.goal_pos[name]
                print ("\n\nCURRENT POS \t{0}".format(pos))
                print ("\nGOAL_POS \t{0}".format(goal))
                vv = np.array(goal) - np.array(pos)
                dist = np.linalg.norm(vv)
                direction = vv / dist

                print ("\nCURRENT ANGLE \t{0}".format(self.rad_offset[name]))

                # print("dist is {0}".format(dist))
                if dist > 10:
                    # print("should move")
                    # write it as a loop here. later reimplement it using many variables outside.
                    initial_pos = pos
                    desired_dir = direction
                    command_dir = rotate_around_with_radians(desired_dir, (0, 0, 1), self.rad_offset[name])
                    print ("\nCOMMAND DIR \t{0}".format(command_dir))

                    # try to move
                    pub = rospy.Publisher("/{0}/cmd_vel".format(name), Twist, latch=True, queue_size=5)

                    reached = False

                    # move for two secs while dist is still large
                    num_iterations = 2 * 60 / rate_fps
                    num_iterations = 10  # override
                    for i in range(0, num_iterations):
                        print("command_dir {0} {1}".format(name, command_dir))
                        msg = Twist()
                        msg.linear.x = command_dir[0] * speed
                        msg.linear.y = command_dir[1] * speed
                        msg.linear.z = command_dir[2] * speed
                        pub.publish(msg)
                        rate.sleep()

                        dist = np.linalg.norm(np.array(goal) - np.array(self.current_pos[name]))

                        if dist < 50:
                            reached = True
                            break

                    rospy.sleep(2) # sleep before deciding on direction change

                    dist = np.linalg.norm(np.array(goal) - np.array(self.current_pos[name]))
                    if dist < 50:
                        reached = True

                    if reached:
                        print("\n\n\n\n\n\n\n\n**************** reached ***************\n\n\n\n\n\n\n\n\n\n")
                    else:
                        print("did not reach")
                        # see how you moved
                        new_pos = self.current_pos[name]
                        actual_dir = np.array(new_pos) - np.array(initial_pos)
                        # update the angle of rotation

                        print("actual_dir {0}".format(actual_dir))

                        # can try command_dir without +=
                        rad_error = radians_from_to_around(actual_dir, desired_dir, (0, 0, 1))
                        self.rad_offset[name] += rad_error
                        print("rad_error:{0} new rad_offset:{1}".format(rad_error, self.rad_offset[name]))

                    print("sleep 5")

                    print ("\n\nCURRENT POS")
                    print (self.current_pos[name])
                    print ("\nGOAL_POS")
                    print (self.goal_pos[name])
                    print ("\n\n")

                    rospy.sleep(5)


                    # calculate angle and fix the coord sys. retry with the new loc.



            rate.sleep()


    def main(self):
        rospy.init_node('balls_controller', anonymous=True)

        self.collect_sphero_names()

        self.listen_to_topics()

        self.move_towards_goals()



if __name__ == '__main__':
    try:
        BallsController().main()

    except rospy.ROSInterruptException:
        pass