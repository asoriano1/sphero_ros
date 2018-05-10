#! /usr/bin/env python2

'''  ______________________________________________________________________
	/*
	*\	This code is part of the sphero_ros project.
	*/
	*\ 	Essentially, we are using the geometry_msgs/Twist msgs to send
	*/ 	velocity commands to the sphero robots
	*\
	*/
	*/	Author: Olalekan Ogunmolu
	*\	Date: April 11, 2017
	*/__________________________________________________________________________
'''

import rospy
import tqdm
from geometry_msgs.msg import Twist
from std_msgs.msg import ColorRGBA, Float32, Bool
from multiprocessing.dummy import Pool as Thread
import numpy as np
import numpy.random as npr

class MoveSphero(object):

	def __init__(self, ns ='', twist = True, pub_color=False, back_led=False, stabilize=True,
				 heading=False, angular=False):
		super(MoveSphero, self).__init__()
		# self.ns = ns
		# self.publishers(ns = self.ns)
		self.namespaces = [ '/sphero_rpg',
							'/sphero_ypp',
							'/sphero_www',
							'/sphero_wwb',
							'/sphero_rwp',
							'/sphero_rgo',
							'/sphero_pob',
							'/sphero_gbr']
		#make pool of workers
		self.pool_workers = Thread(4)

		self.twist     = twist
		self.pub_color = pub_color
		self.back_led	= back_led
		self.stabilize	= stabilize
		self.heading	= heading
		self.angular	= angular
		self.def_queue_size = 5

		if self.twist == True:
			self.twist_rgo_pub = rospy.Publisher('/sphero_rgo/cmd_vel', Twist, queue_size = self.def_queue_size)
			self.twist_ypp_pub = rospy.Publisher('/sphero_ypp/cmd_vel', Twist, queue_size = self.def_queue_size)
			self.twist_pob_pub = rospy.Publisher('/sphero_pob/cmd_vel', Twist, queue_size = self.def_queue_size)
			self.twist_www_pub = rospy.Publisher('/sphero_www/cmd_vel', Twist, queue_size = self.def_queue_size)
			self.twist_rpg_pub = rospy.Publisher('/sphero_rpg/cmd_vel', Twist, queue_size = self.def_queue_size)
			self.twist_rwp_pub = rospy.Publisher('/sphero_rwg/cmd_vel', Twist, queue_size = self.def_queue_size)
			self.twist_wwb_pub = rospy.Publisher('/sphero_wwb/cmd_vel', Twist, queue_size = self.def_queue_size)
			self.twist_gbr_pub = rospy.Publisher('/sphero_gbr/cmd_vel', Twist, queue_size = self.def_queue_size)

		if self.pub_color:
			self.color_rgo_pub = rospy.Publisher('/sphero_rgo/set_color', ColorRGBA,  queue_size = self.def_queue_size)
			self.color_ypp_pub = rospy.Publisher('/sphero_ypp/set_color', ColorRGBA,  queue_size = self.def_queue_size)
			self.color_pob_pub = rospy.Publisher('/sphero_pob/set_color', ColorRGBA,  queue_size = self.def_queue_size)
			self.color_www_pub = rospy.Publisher('/sphero_www/set_color', ColorRGBA,  queue_size = self.def_queue_size)
			self.color_rpg_pub = rospy.Publisher('/sphero_rpg/set_color', ColorRGBA,  queue_size = self.def_queue_size)
			self.color_rwg_pub = rospy.Publisher('/sphero_rwg/set_color', ColorRGBA,  queue_size = self.def_queue_size)
			self.color_wwb_pub = rospy.Publisher('/sphero_wwb/set_color', ColorRGBA,  queue_size = self.def_queue_size)
			self.color_gbr_pub = rospy.Publisher('/sphero_gbr/set_color', ColorRGBA,  queue_size = self.def_queue_size)

		if self.back_led:
			self.back_rgo_pub = rospy.Publisher('/sphero_rgo/set_back_led', Float32,  queue_size = self.def_queue_size)
			self.back_ypp_pub = rospy.Publisher('/sphero_ypp/set_back_led', Float32,  queue_size = self.def_queue_size)
			self.back_pob_pub = rospy.Publisher('/sphero_pob/set_back_led', Float32,  queue_size = self.def_queue_size)
			self.back_www_pub = rospy.Publisher('/sphero_www/set_back_led', Float32,  queue_size = self.def_queue_size)
			self.back_rpg_pub = rospy.Publisher('/sphero_rpg/set_back_led', Float32,  queue_size = self.def_queue_size)
			self.back_rwg_pub = rospy.Publisher('/sphero_rwg/set_back_led', Float32,  queue_size = self.def_queue_size)
			self.back_wwb_pub = rospy.Publisher('/sphero_wwb/set_back_led', Float32,  queue_size = self.def_queue_size)
			self.back_gbr_pub = rospy.Publisher('/sphero_gbr/set_back_led', Float32,  queue_size = self.def_queue_size)

		if self.stabilize:
			self.stab_rgo_pub = rospy.Publisher('/sphero_rgo/disable_stabilization', Bool,  queue_size = self.def_queue_size)
			self.stab_ypp_pub = rospy.Publisher('/sphero_ypp/disable_stabilization', Bool,  queue_size = self.def_queue_size)
			self.stab_pob_pub = rospy.Publisher('/sphero_pob/disable_stabilization', Bool,  queue_size = self.def_queue_size)
			self.stab_www_pub = rospy.Publisher('/sphero_www/disable_stabilization', Bool,  queue_size = self.def_queue_size)
			self.stab_rpg_pub = rospy.Publisher('/sphero_rpg/disable_stabilization', Bool,  queue_size = self.def_queue_size)
			self.stab_rwg_pub = rospy.Publisher('/sphero_rwg/disable_stabilization', Bool,  queue_size = self.def_queue_size)
			self.stab_wwb_pub = rospy.Publisher('/sphero_wwb/disable_stabilization', Bool,  queue_size = self.def_queue_size)
			self.stab_gbr_pub = rospy.Publisher('/sphero_gbr/disable_stabilization', Bool,  queue_size = self.def_queue_size)

		if self.heading:
			self.head_rgo_pub = rospy.Publisher('/sphero_rgo/set_heading', Float32,  queue_size = self.def_queue_size)
			self.head_ypp_pub = rospy.Publisher('/sphero_ypp/set_heading', Float32,  queue_size = self.def_queue_size)
			self.head_pob_pub = rospy.Publisher('/sphero_pob/set_heading', Float32,  queue_size = self.def_queue_size)
			self.head_www_pub = rospy.Publisher('/sphero_www/set_heading', Float32,  queue_size = self.def_queue_size)
			self.head_rpg_pub = rospy.Publisher('/sphero_rpg/set_heading', Float32,  queue_size = self.def_queue_size)
			self.head_rwg_pub = rospy.Publisher('/sphero_rwg/set_heading', Float32,  queue_size = self.def_queue_size)
			self.head_wwb_pub = rospy.Publisher('/sphero_wwb/set_heading', Float32,  queue_size = self.def_queue_size)
			self.head_gbr_pub = rospy.Publisher('/sphero_gbr/set_heading', Float32,  queue_size = self.def_queue_size)

		if self.angular:
			self.angvel_rgo_pub = rospy.Publisher('/sphero_rgo/set_angular_velocity', Float32, queue_size = self.def_queue_size)
			self.angvel_ypp_pub = rospy.Publisher('/sphero_ypp/set_angular_velocity', Float32, queue_size = self.def_queue_size)
			self.angvel_pob_pub = rospy.Publisher('/sphero_pob/set_angular_velocity', Float32, queue_size = self.def_queue_size)
			self.angvel_www_pub = rospy.Publisher('/sphero_www/set_angular_velocity', Float32, queue_size = self.def_queue_size)
			self.angvel_rpg_pub = rospy.Publisher('/sphero_rpg/set_angular_velocity', Float32, queue_size = self.def_queue_size)
			self.angvel_rwg_pub = rospy.Publisher('/sphero_rwg/set_angular_velocity', Float32, queue_size = self.def_queue_size)
			self.angvel_wwb_pub = rospy.Publisher('/sphero_wwb/set_angular_velocity', Float32, queue_size = self.def_queue_size)
			self.angvel_gbr_pub = rospy.Publisher('/sphero_gbr/set_angular_velocity', Float32, queue_size = self.def_queue_size)

	def sphero_populate(self):
		# generate velocity commads
		twist_msg, twist_msg_2 = Twist(), Twist()
		twist_msg_3, twist_msg_4 = Twist(), Twist()

		twist_msg.linear.x = 100*npr.random_sample()
		twist_msg.linear.y = 50*npr.random_sample()
		twist_msg.linear.z = 32*npr.random_sample()
		twist_msg.angular.x = 100*npr.random_sample()
		twist_msg.angular.y = 200*npr.random_sample()
		twist_msg.angular.z = 300*npr.random_sample()

		twist_msg_2.linear.x = 2.2*twist_msg.linear.x
		twist_msg_2.linear.y = 2.2*twist_msg.linear.y
		twist_msg_2.linear.z = 2.2*twist_msg.linear.z
		twist_msg_2.angular.x = 2.2*twist_msg.angular.x
		twist_msg_2.angular.y = 2.2*twist_msg.angular.y
		twist_msg_2.angular.z = 2.2*twist_msg.angular.z

		twist_msg_3.linear.x = 0.5*twist_msg.linear.x
		twist_msg_3.linear.y = 0.5*twist_msg.linear.y
		twist_msg_3.linear.z = 0.5*twist_msg.linear.z
		twist_msg_3.angular.x = 0.5*twist_msg.angular.x
		twist_msg_3.angular.y = 0.5*twist_msg.angular.y
		twist_msg_3.angular.z = 0.5*twist_msg.angular.z

		twist_msg_4.linear.x = 0.25*twist_msg_2.linear.x
		twist_msg_4.linear.y = 0.25*twist_msg_2.linear.y
		twist_msg_4.linear.z = 0.25*twist_msg_2.linear.z
		twist_msg_4.angular.x = 0.25*twist_msg_2.angular.x
		twist_msg_4.angular.y = 0.25*twist_msg_2.angular.y
		twist_msg_4.angular.z = 0.25*twist_msg_2.angular.z

		#send robots around on twist topic
		self.twist_rgo_pub.publish(twist_msg)
		self.twist_ypp_pub.publish(twist_msg_2)
		self.twist_pob_pub.publish(twist_msg_3)
		self.twist_www_pub.publish(twist_msg_4)

		self.twist_rpg_pub.publish(twist_msg)
		self.twist_rwp_pub.publish(twist_msg_2)
		self.twist_wwb_pub.publish(twist_msg_3)
		self.twist_gbr_pub.publish(twist_msg_4)

if __name__ == '__main__':
	try:
		rospy.init_node('sphero_command_center', anonymous=True)

		ms = MoveSphero()

		rate = rospy.Rate(10) # 10hz

		while not rospy.is_shutdown():
			ms.sphero_populate()
			rate.sleep()

	except rospy.ROSInterruptException:
		pass
