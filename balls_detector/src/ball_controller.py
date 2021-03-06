#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from pyquaternion import Quaternion


def radians_from_to_around(v1, v2, a):
    c = np.cross(v1, v2)

    d = np.dot(v1, v2)

    radians = np.arctan2(np.dot(c, a), d)
    print("ANGLE FROM \t{0} TO \t{1} IS \t{2}".format(v1, v2, radians * 180 / np.pi))
    return radians


def rotate_around_with_radians(v1, a, radians):
    quat = Quaternion(axis=a, radians=radians)

    return quat.rotate(v1)


def to_command_frame(dir_vec):
    return np.multiply(dir_vec, [-1, 1, 1])


class BallController:
    def __init__(self):
        self.sphero_name = ""
        # self.sphero_names = None
        self.current_pos = None
        self.goal_pos = None
        self.rad_offset = 0

    def listen_to_topics(self):
        # desired position
        goal_pos_topic = "/{0}/cam_image_goal_pos".format(self.sphero_name)
        rospy.Subscriber(goal_pos_topic, Float64MultiArray, self.goal_pos_detected)

        # current position
        current_pos_topic = "/{0}/cam_image_pos".format(self.sphero_name)
        rospy.Subscriber(current_pos_topic, Float64MultiArray, self.current_pos_detected)

    def goal_pos_detected(self, msg):
        print("goal pos detected {0}".format(msg.data))
        self.goal_pos = list(msg.data) + [0]  # copy

    def current_pos_detected(self, msg):
        self.current_pos = list(msg.data) + [0]  # copy

    def move_towards_goal(self):
        rate_fps = 10
        speed = 20
        dist_close_enough = 50

        while not rospy.is_shutdown():
            if self.goal_pos is None or self.current_pos is None:
                rospy.sleep(1)
            else:

                print("\nWILL MOVE {0}".format(self.sphero_name))
                pos = self.current_pos
                goal = self.goal_pos
                print ("\n\nCURRENT POS \t{0}".format(pos))
                print ("\nGOAL_POS \t{0}".format(goal))
                vv = np.array(goal) - np.array(pos)
                dist = np.linalg.norm(vv)
                direction = vv / dist

                print ("\nCURRENT ANGLE \t{0}".format(self.rad_offset))

                # print("dist is {0}".format(dist))
                if dist > dist_close_enough:
                    # print("should move")
                    # write it as a loop here. later reimplement it using many variables outside.
                    initial_pos = pos
                    desired_dir = direction
                    corrected_desired_dir = rotate_around_with_radians(desired_dir, (0, 0, 1), self.rad_offset)
                    command_dir = to_command_frame(corrected_desired_dir)

                    print("\n\nDESIRED, CORRECTED, COMMAND: {0}, {1}, {2}".format(desired_dir, corrected_desired_dir, command_dir))

                    print ("\nCOMMAND DIR \t{0}".format(command_dir))

                    reached = self.move_one_step(command_dir, self.sphero_name, rate_fps, speed, 2, dist_close_enough, goal)

                    if reached:
                        print("\n\n\n\n\n\n\n\n**************** reached ***************\n\n\n\n\n\n\n\n\n\n")
                    else:
                        print("did not reach")

                        # rospy.sleep(3) #trying online

                        # see how you moved
                        new_pos = self.current_pos
                        actual_dir = np.array(new_pos) - np.array(initial_pos)
                        # update the angle of rotation

                        print("actual_dir {0}".format(actual_dir))

                        # can try command_dir without +=
                        rad_error = radians_from_to_around(actual_dir, desired_dir, (0, 0, 1))

                        # before you update the rad error, move one step back. maybe this time it will go to target.
                        # this should help me debug my code
                        # self.move_one_step(-1 * command_dir, name, rate_fps, speed)
                        # TODO test it like this and see if going back and moving again moves better.

                        self.rad_offset += rad_error
                        print("rad_error:{0} new rad_offset:{1}".format(rad_error, self.rad_offset))

                        # move a bit more before recording the new initial_pos
                        corrected_desired_dir = rotate_around_with_radians(desired_dir, (0, 0, 1), self.rad_offset)
                        command_dir = to_command_frame(corrected_desired_dir)
                        self.move_one_step(command_dir, self.sphero_name, rate_fps, speed, 1, dist_close_enough, goal)


                    # ok now. want to do online fix
                    # remember a location that you are sure the current direction started.
                    # go for a while. take a snapshot. do a correction with the start location. apply the correction.
                    # update the start location a bit later as the new command will propagate.

                    print ("\n\nCURRENT POS")
                    print (self.current_pos)
                    print ("\nGOAL_POS")
                    print (self.goal_pos)
                    print ("\n\n")



                    # calculate angle and fix the coord sys. retry with the new loc.

            rate = rospy.Rate(rate_fps)
            rate.sleep()

    def move_one_step(self, command_dir, name, rate_fps, speed, duration, dist_close_enough = None, goal = None):
        # try to move
        rate = rospy.Rate(rate_fps)
        pub = rospy.Publisher("/{0}/cmd_vel".format(name), Twist, latch=True, queue_size=5)
        reached = False
        # move for two secs while dist is still large
        # num_iterations = 2 * 60 / rate_fps
        num_iterations = duration * rate_fps  # TODO make this a loop in time instead

        print ("duration {0}".format(duration))
        print ("\n\n\n\nnum_iterations {0}\n\n\n\n\n\n".format(num_iterations))
        for i in range(0, num_iterations):
            print("command_dir {0}".format(name, command_dir))
            msg = Twist()
            msg.linear.x = command_dir[0] * speed
            msg.linear.y = command_dir[1] * speed
            msg.linear.z = command_dir[2] * speed
            pub.publish(msg)
            rate.sleep()

            if goal is not None:
                if np.linalg.norm(np.array(goal) - np.array(self.current_pos)) < dist_close_enough:
                    reached = True
                    break

        return reached
        # will try online now
        # rospy.sleep(2)  # sleep before deciding on direction change
        #
        # if goal is not None:
        #     dist = np.linalg.norm(np.array(goal) - np.array(self.current_pos[name]))
        #     if dist < 50:
        #         reached = True
        #     return reached

    def main(self):
        rospy.init_node('ball_controller', anonymous=True)

        self.sphero_name = rospy.get_param("~sphero_name", self.sphero_name)

        if len(self.sphero_name) > 0:
            self.listen_to_topics()

            self.move_towards_goal()



if __name__ == '__main__':
    try:
        BallController().main()

    except rospy.ROSInterruptException:
        pass
