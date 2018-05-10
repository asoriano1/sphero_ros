#!/usr/bin/env python2
#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2012, Melonee Wise
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#***********************************************************
#author: Melonee Wise

import rospy

import math
import os, sys

from sensor_msgs.msg import Imu, Joy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TwistWithCovariance, Vector3
from sphero_node.msg import SpheroCollision
from std_msgs.msg import ColorRGBA, Float32, Bool

class sphero( ):
	def __init__(self,ns = ''):
		self.ns = ns
		self.init_pubsub(ns = self.ns)
		if ns is not '':
			param = '%s/%s/connect_'%(ns,ns)
		r = rospy.get_param('%sred'%param,0)
		g = rospy.get_param('%sgreen'%param,255)
		b = rospy.get_param('%sblue'%param,0)
		self.connect_color = (r,g,b)
		self.control_color = (255,100,0)
	def callback_odometry(self, message):
		pass#rospy.loginfo('Odometry Message')
	def callback_imu(self, message):
		pass#rospy.loginfo('IMU Message')
	def callback_collision(self, message):
		rospy.loginfo('Collision detected: (%d,%d,%d)@%d-(%f,%f):%f'%(
			message.x,
			message.y,
			message.z,
			message.axis,
			message.x_magnitude,
			message.y_magnitude,
			message.speed))

	def cmd_vel(self,x,y):
		twist = Twist()
		twist.linear.x = x
		twist.linear.y = y
		self.cmd_vel_pub.publish(twist)

	def set_color(self,color):
		_color = ColorRGBA()
		_color.r=float(color[0]/255.0)
		_color.g=float(color[1]/255.0)
		_color.b=float(color[2]/255.0)
		self.color_pub.publish(_color)

	def init_pubsub(self, ns='',def_queue_size = 1):
		self.odom_sub = rospy.Subscriber('%sodom'%ns, Odometry , self.callback_odometry, queue_size=def_queue_size)
		self.imu_sub = rospy.Subscriber('%simu'%ns, Imu, self.callback_imu, queue_size=def_queue_size)
		self.collision_sub = rospy.Subscriber('%scollision'%ns, SpheroCollision , self.callback_collision, queue_size=def_queue_size)
		#self.diag_sub = rospy.Subscriber('/diagnostics', DiagnosticArray , queue_size=def_queue_size)

		self.cmd_vel_pub = rospy.Publisher('%scmd_vel'%ns, Twist, queue_size = def_queue_size)
		self.color_pub = rospy.Publisher('%sset_color'%ns, ColorRGBA,  queue_size = def_queue_size)
		self.back_led_pub = rospy.Publisher('%sset_back_led'%ns, Float32,  queue_size = def_queue_size)
		self.stabilization_pub = rospy.Publisher('%sdisable_stabilization'%ns, Bool,  queue_size = def_queue_size)
		self.heading_pub = rospy.Publisher('%sset_heading'%ns, Float32,  queue_size = def_queue_size)
		self.angular_velocity_pub = rospy.Publisher('%sset_angular_velocity'%ns, Float32, queue_size = def_queue_size)

class joystick():
	def __init__(self,spheros,deadzone = 0.05):
		self.joy_sub = rospy.Subscriber('joy', Joy , self.callback_joystick, queue_size=10)
		self.spheros = spheros
		self.active_sphero = spheros[0]
		self.deadzone = deadzone
		self.axes_gain=(100,100,1,1,1,1,1,1)
	def callback_joystick(self,message):
		#rospy.loginfo('Joystick Message')
		if message.buttons[0]:
			self.next_sphero()
		axes = []
		for axis in message.axes:
			axes.append(self.apply_deadzone(axis))
		self.active_sphero.cmd_vel(axes[0]*self.axes_gain[0],axes[1]*self.axes_gain[1])
			
		
	def apply_deadzone(self,val):
		if val > 0 and val > self.deadzone:
			return val
		if val < 0 and val < -self.deadzone:
			return val
		return 0
	def next_sphero(self):
		index = self.spheros.index(self.active_sphero) + 1
		if index > len(self.spheros)-1:
			index = 0
		return self.control_sphero(index)

	def control_sphero(self,sphero_num):
		self.active_sphero.set_color(self.active_sphero.connect_color)
		self.active_sphero = self.spheros[sphero_num]
		self.active_sphero.set_color(self.active_sphero.control_color)
		return self.active_sphero

def get_spheros():
	name = rospy.get_name()	
	num_spheros = rospy.get_param('%s/num_spheros'%name,1)
	spheros=[]
	for i in range(1,num_spheros+1):
		ns = 'sphero%d/'%(i)
		spheros.append(sphero(ns))
	return spheros

if __name__ == '__main__':
	rospy.init_node('sphero_control')
	subprocess = os.popen('sudo xboxdrv -d --silent','w',65536)
	subprocess.write('nasa\n')
	spheros = get_spheros()
	controller = joystick(spheros)
	'''
        r = rospy.Rate(0.50)
        while not rospy.is_shutdown():
		for sphero in spheros:
			rospy.loginfo('Commanding %s...'%sphero.ns)
			sphero.cmd_vel(100,100)
		r.sleep()
	'''
	rospy.spin()


