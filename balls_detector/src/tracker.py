#!/usr/bin/env python2

'''
we subscribe to the published centroids (/locs/tagged & /locs/detected)
and rearrange the new centroids according to a bijective nearest neighbor rule,
then publish the arranged data to a new topic (/locs/ordered)

Authors:
    1.  Sleiman Safaoui
    2.  Kaveh Fathian
    3.  Olalekan Ogunmolu

July 16, 2017
'''

from __future__ import print_function
import numpy as np
import sys

#ros imports
import rospy
import roslib
roslib.load_manifest('balls_detector')
from std_msgs.msg import String
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import Float64MultiArray


class TagsGrabber:
    '''
    We get the correctly ordered centroids published after tagger.cpp runs, and
    set that order to be the default order (based on which we will track)
    '''
    tags_new = []
    def __init__(self):
        self.tag_sub = rospy.Subscriber("/locs/tagged", Float64MultiArray,
                                        self.callbackT, queue_size=1)

    def callbackT(self, data):
        print("\nRECEIVED TAGGED:")
        print(data)
        self.tags_new = data

    def get_tags_new(self):
        return self.tags_new

class PosGrabber:
    '''
    We get the newly published centroids.
    '''
    pos_new = []
    def __init__(self):
        self.pos_sub = rospy.Subscriber("/locs/detected", Float64MultiArray,
                                        self.callbackP, queue_size=1)

    def callbackP(self, data):
        print("\nRECEIVED DETECTED:")
        print(data)
        self.pos_new = data

    def get_pos_new(self):
        return self.pos_new

class Tracker(TagsGrabber, PosGrabber):
    '''
    We rearrange the newly detected centroids to maintain the order of the robots
    as detected in the first frame.
    '''
    def __init__(self):
        TagsGrabber.__init__(self)
        PosGrabber.__init__(self)
        self.pos_pub = rospy.Publisher("/locs/ordered", Float64MultiArray,
                                        queue_size = 100)

    def delay(self, pos_old, pos_ordered): # delay ordering to allow for proper initialization
        pos_ordered = self.get_tags_new()
        pos_old = pos_ordered
        rospy.sleep(0.2)
        return pos_old, pos_ordered

    def order(self, first, pos_old, pos_ordered):
        pos_new = self.get_pos_new()
        n_old = len(pos_old.data)/2
        n_new = len(pos_new.data)/2
        if n_new >= n_old: # no occluded robot(s)
            pos_ordered = self.nn_search( n_old, n_new, pos_old, pos_ordered )#######
        else: # robot(s) occluded keep pos_old (nothing new to publish)
            pos_ordered = pos_old
        pos_old = pos_ordered # set ordered pos to be old ones after publishing
        return (pos_old, pos_ordered)

    def nn_search(self, n_old, n_new, pos_old, pos_ordered ):
        print("\n***NN SEARCH ")
        print(n_old)
        print(n_new)
        print(pos_old)
        print(pos_ordered)
        dist = np.zeros(n_old*n_new) # distance between points
        idx = np.zeros((2,n_old*n_new)) # index of pairs of points (old and new) corresponding to distances in dist
        pt_idx = np.zeros(n_old) # indicies of pos_new matched to pos_old in the latter's order

        # find distance between all pairs
        itr = 0
        for i in range(0,n_old):
            for j in range(0,n_new):
                dist[itr] = ( (pos_old.data[int(2*i)] - self.pos_new.data[int(2*j)])**2 +
                            (pos_old.data[int(2*i+1)] - self.pos_new.data[int(2*j+1)])**2 )
                idx[0][itr] = i
                idx[1][itr] = j
                itr +=1
        pos_ordered_temp = np.zeros(int(2*n_old)) # temporary array to store pos_ordered

        # order according to bijective nearest neighbor search
        for i in range(0, n_old):
            s_idx = sorted(range(len(dist)), key=lambda k: dist[k]) #  ascending oreder sort index
            new_pt_idx = idx[1][s_idx[0]] # index of new point with shortest distance
            old_pt_idx = idx[0][s_idx[0]] # index of corresponding old point (with shortest distance)
            # points pos_new(new_pt_idx) and pos_old(old_pt_idx) are paired as being the closes neighbors

            # store pos_new(new_pt_idx) at the position old_pt_idx
            # (thus, rearranging the order of pos_new to be the same as pos_old)
            pos_ordered_temp[int(old_pt_idx*2)] = self.pos_new.data[int(new_pt_idx*2)]
            pos_ordered_temp[int(old_pt_idx*2+1)] = self.pos_new.data[int(new_pt_idx*2+1)]
            pt_idx[i] = new_pt_idx

            # remove all pairs in idx and dist that correspond to
            # pos_new(new_pt_idx) or pos_old(old_pt_idx)
            rem_idx = np.zeros(len(dist))
            for j in range(len(rem_idx)):
                rem_idx[j] = (idx[1][j] == new_pt_idx) or (idx[0][j] == old_pt_idx)
            for j in range(len(rem_idx)-1, -1, -1):
                if rem_idx[j] == 1:
                    idx = np.delete(idx, j, 1)
                    dist = np.delete(dist, j)

        pos_ordered.data = pos_ordered_temp
        self.pos_pub.publish(pos_ordered)
        print("\n***ORDERED")
        print(pos_ordered)
        return pos_ordered

def main():
    rospy.init_node("tracked_pos")
    rate = rospy.Rate(30)
    tr = Tracker()
    num_rob = rospy.get_param("/tracker/Rob_Params/num_rob")
    first = 30*(3 + 2*num_rob) # wait loops for proper initialization of pos_old and pos_ordered
    pos_old = []
    pos_ordered = []
    try:
        while not rospy.is_shutdown():
            if not (first == 0):
                (pos_old, pos_ordered) = tr.delay(pos_old, pos_ordered)
                print("\n***AFTER DELAY: ")
                print(pos_old)
                print(pos_ordered)
                print(first)
                first -=1
            else: # initialization of pos_old and pos_ordered complete --> track
                (pos_old, pos_ordered) = tr.order(first, pos_old, pos_ordered)
                print("\n***ORDER: ")
                print(pos_old)
                print(pos_ordered)
            rate.sleep()
    except KeyboardInterrupt:
        print("shutting down ros")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
