#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (c) 2015 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND ISC DISCLAIMS ALL WARRANTIES WITH
# REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL ISC BE LIABLE FOR ANY
# SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
# OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Authors:
# * Jeremie Deray

import rospy
import roslib
import tf
import numpy as np
import sys, getopt
from geometry_msgs.msg import Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from math import radians, floor

import os.path
import rosbag

import params

class carmen2rosbag:
	
	def __init__(self):
		
		self.LASER_MESSAGE_DEFINED       = ["RAWLASER1", "RAWLASER2", "RAWLASER3", "RAWLASER4"]
		self.OLD_LASER_MESSAGE_DEFINED   = ["FLASER", "RLASER", "LASER3", "LASER4"]
		self.ROBOT_LASER_MESSAGE_DEFINED = ["ROBOTLASER1", "ROBOTLASER2"]
		self.GPS_MESSAGE_DEFINED         = ["NMEAGGA", "NMEARMC"]
		self.ODOM_DEFINED 				 =  "ODOM"
		self.TRUEPOS_DEFINED 			 =  "TRUEPOS"
		self.PARAM_DEFINED               =  "PARAM"
		self.SYNC_DEFINED	             =  "SYNC"
		
		self.unknown_entries = []
		
		self.stamp = rospy.get_rostime()
		
		self.pose_msg  	   = Odometry()
		self.true_pose_msg = Odometry()
		
		self.laser_msg = LaserScan()

		self.tf_odom_robot_msg   = TransformStamped()
		self.tf_laser_robot_msg  = TransformStamped()
		self.tf2_msg 			 = TFMessage()
		
		self.tf_tr = tf.Transformer(True, rospy.Duration(2.0))
		
		self.pause = False

		self.rate = rospy.get_param("~global_rate",  40)

		RAWLASER1_topic   = rospy.get_param("~RAWLASER1_topic", "/RAWLASER1")
		RAWLASER2_topic   = rospy.get_param("~RAWLASER2_topic", "/RAWLASER2")
		RAWLASER3_topic   = rospy.get_param("~RAWLASER3_topic", "/RAWLASER3")
		RAWLASER4_topic   = rospy.get_param("~RAWLASER4_topic", "/RAWLASER4")
		
		FLASER_topic 	  = rospy.get_param("~FLASER_topic", "/FLASER")
		RLASER_topic 	  = rospy.get_param("~RLASER_topic", "/RLASER")
		LASER3_topic 	  = rospy.get_param("~LASER3_topic", "/LASER3")
		LASER4_topic 	  = rospy.get_param("~LASER4_topic", "/LASER4")
		
		ROBOTLASER1_topic = rospy.get_param("~ROBOTLASER1_topic", "/ROBOTLASER1")
		ROBOTLASER2_topic = rospy.get_param("~ROBOTLASER2_topic", "/ROBOTLASER2")
		
		NMEAGGA_topic 	  = rospy.get_param("~NMEAGGA_topic", "/NMEAGGA")
		NMEARMC_topic 	  = rospy.get_param("~NMEARMC_topic", "/NMEARMC")
		
		ODOM_topic 	  	  = rospy.get_param("~ODOM_topic", "/ODOM")
		TRUEPOS_topic 	  = rospy.get_param("~TRUEPOS_topic", "/TRUEPOS")
		
		tf_topic      	  = rospy.get_param("~tf_topic", "/tf")
		
		robot_link  	  = rospy.get_param("~robot_link", "base_link")
		odom_link   	  = rospy.get_param("~odom_link", "odom")
		odom_robot_link	  = rospy.get_param("~odom_robot_link", "odom_robot_link")
		true_odom_link 	  = rospy.get_param("~trueodom_link", "gt_odom")
		ROBOTLASER1_link  = rospy.get_param("~ROBOTLASER1_link", "ROBOTLASER1_link")
		ROBOTLASER2_link  = rospy.get_param("~ROBOTLASER2_link", "ROBOTLASER2_link")
		
		# Hacky param to choose which one of ODOM logs or ROBOTLASER1 to publish on tf.
		# If set to False, ODOM is pub on tf
		# If set to True,  ROBOTLASER1 is pub on tf
		# Must be careful, if set to True and there are no ROBOTLASER1 but a ODOM
		# There won't be anything on tf ...
		# TODO : - less hacky way
		#		 - choose which one of ROBOTLASER1 or ROBOTLASER2 etc
		self.publish_corrected = rospy.get_param("~pub_corrected", False)

		self.topics = {"RAWLASER1" 	 : RAWLASER1_topic,
					   "RAWLASER2" 	 : RAWLASER2_topic,
					   "RAWLASER3" 	 : RAWLASER3_topic,
					   "RAWLASER4" 	 : RAWLASER4_topic,
					   "FLASER" 	 : FLASER_topic,
					   "RLASER" 	 : RLASER_topic,
					   "LASER3" 	 : LASER3_topic,
					   "LASER4" 	 : LASER4_topic,
					   "ROBOTLASER1" : ROBOTLASER1_topic,
					   "ROBOTLASER2" : ROBOTLASER2_topic,
					   "NMEAGGA" 	 : NMEAGGA_topic,
					   "NMEARMC" 	 : NMEARMC_topic,
					   "ODOM" 		 : ODOM_topic,
					   "TRUEPOS" 	 : TRUEPOS_topic,	
					   "TF" 		 : tf_topic}

		self.links = {"ROBOT" 		: robot_link,
					  "ODOM" 		: odom_link,
					  "ROBOTODOM"	: odom_robot_link,
					  "TRUEPOS" 	: true_odom_link,
					  "ROBOTLASER1" : ROBOTLASER1_link,
					  "ROBOTLASER2" : ROBOTLASER2_link }

		self.params = {"laser" 	  : params.old_laser_params,
					   "newlaser" : params.new_laser_params,
					   "robot"    : params.robot_params,
					   "gps"      : params.gps_params,
					   "base"     : params.base_params,
					   "arm"      : params.arm_params,
					   "segway"   : params.segway_params} #TOfinish

	def __exit__(self, type, value, traceback):
		self.close_bag()

	def convert(self, inputfile, outputfile):

		#TODO : deal with exceptions
		try:
			inCompleteName = os.path.expanduser(inputfile)
			self.f = open(inCompleteName, "r")
		except (IOError, ValueError):
			rospy.logerr("Couldn't open %", inputfile)
			self.print_help()
			exit(-1)

		try:
			outCompleteName = os.path.expanduser(outputfile)
			self.bag = rosbag.Bag(outCompleteName, "w")
		except (IOError, ValueError):
			rospy.logerr("Couldn't open %", outputfile)
			self.print_help()
			exit(-1)

		rospy.loginfo("Reading data from : %s", inCompleteName)
		rospy.loginfo("Writing rosbag to : %s", outCompleteName)
		rospy.loginfo("Start convertion to rosbag")

		self.counter = 0

		for line in self.f:
			words = line.split()

			if self.pause == True:
				while self.pause == True:
					rospy.sleep(1/self.rate)				

			if self.LASER_MESSAGE_DEFINED.count(words[0]):
				
				self.fillUpLaserMessage(words)
				
				topic = self.topics[words[0]]
				self.bag.write(topic, self.laser_msg, self.laser_msg.header.stamp)
				
				self.laser_msg.header.seq = self.laser_msg.header.seq + 1

			elif self.OLD_LASER_MESSAGE_DEFINED.count(words[0]):
				
				self.fillUpOldLaserMessage(words)
				
				topic = self.topics[words[0]]

				if not self.laser_msg.header.stamp.secs == 0:
					self.bag.write(topic, self.laser_msg, self.laser_msg.header.stamp)

					if self.publish_corrected:
						topic = self.topics["TF"]
						self.bag.write(topic, self.tf2_msg, self.laser_msg.header.stamp)
				
					self.laser_msg.header.seq = self.laser_msg.header.seq + 1
				
			elif self.ROBOT_LASER_MESSAGE_DEFINED.count(words[0]):
				
				self.fillUpRobotLaserMessage(words)
				
				topic = self.topics[words[0]]
				self.bag.write(topic, self.laser_msg, self.laser_msg.header.stamp)
				
				if self.publish_corrected:
					topic = self.topics["TF"]
					self.bag.write(topic, self.tf2_msg, self.laser_msg.header.stamp)

					topic = self.topics["ODOM"]
					self.bag.write(topic, self.pose_msg, self.pose_msg.header.stamp)

					self.pose_msg.header.seq  = self.pose_msg.header.seq  + 1
					self.tf_laser_robot_msg.header.seq = self.tf_laser_robot_msg.header.seq + 1
					self.tf_odom_robot_msg.header.seq  = self.tf_odom_robot_msg.header.seq  + 1

				self.laser_msg.header.seq = self.laser_msg.header.seq + 1
				
			elif self.ODOM_DEFINED == words[0]:
				
				self.fillUpOdomMessage(words)
				
				if not self.pose_msg.header.stamp.secs == 0 and not self.publish_corrected:
					topic = self.topics["ODOM"]
					self.bag.write(topic, self.pose_msg, self.pose_msg.header.stamp)
				
					topic = self.topics["TF"]
					self.bag.write(topic, self.tf2_msg, self.pose_msg.header.stamp)

					self.pose_msg.header.seq = self.pose_msg.header.seq + 1
					self.tf_odom_robot_msg.header.seq = self.tf_odom_robot_msg.header.seq + 1
				
			elif self.TRUEPOS_DEFINED == words[0]:
				
				self.fillUpTruePoseMessage(words)
				
				topic = self.topics[words[0]]
				self.bag.write(topic, self.true_pose_msg, self.true_pose_msg.header.stamp)
				
				topic = self.topics["TF"]
				self.bag.write(topic, self.tf2_msg, self.tf_msg.header.stamp)

				self.pose_msg.header.seq = self.true_pose_msg.header.seq + 1
				
			elif self.PARAM_DEFINED == words[0]:
				
				self.param(words)
			
			else:
				if not self.unknown_entries.count(words[0]):
					self.unknown_entries.append(words[0])
					if not words[0] == "#":
						rospy.logerr("Unknown entry %s !", words[0])
				
			self.increment_stamp()
			self.tf2_msg = TFMessage()
				
	def fillUpLaserMessage(self, words):

		self.laser_msg.header.frame_id = "base_link"
		
		self.laser_msg.angle_increment = float(words[4])
		self.laser_msg.angle_min = float(words[2])
		self.laser_msg.angle_max = float(words[2])+float(words[3])

		self.laser_msg.range_min = 0.
		self.laser_msg.range_max = float(words[5])
				
		ranges = []
		num_range_readings = int(words[8])
		last_range_reading = num_range_readings + 8

		for word in words[9:last_range_reading+1]:
			ranges.append( float( word ))
			
		self.laser_msg.ranges = ranges

		# min-max angle fitting, Karto need
		factor_angle_fitting = self.laser_msg.angle_increment / float(2)
		while (round((self.laser_msg.angle_max - self.laser_msg.angle_min)/self.laser_msg.angle_increment) + 1) != num_range_readings:
			if (round((self.laser_msg.angle_max - self.laser_msg.angle_min)/self.laser_msg.angle_increment) + 1) > num_range_readings:
				self.laser_msg.angle_min = self.laser_msg.angle_min + factor_angle_fitting
			else
				self.laser_msg.angle_max = self.laser_msg.angle_max - factor_angle_fitting

			factor_angle_fitting = factor_angle_fitting / float(2)

		ranges = []
		num_emisison_readings = int(words[last_range_reading+1])
		last_emission_reading = num_emisison_readings + last_range_reading
		
		for word in words[last_range_reading+1:last_emission_reading+1]:
			ranges.append( float( word ))

		self.laser_msg.intensities = ranges

		self.laser_msg.header.stamp = rospy.Time( float(words[last_emission_reading+2]) )
		
	def fillUpOldLaserMessage(self, words):
		
		self.laser_msg.header.frame_id = "base_link"		
		ranges = []
		num_range_readings = int(words[1])
		last_range_reading = num_range_readings + 1
		for word in words[2:last_range_reading+1]:					
			ranges.append( float( word ) )

		laser_id = params.laserId2ParamPrefix(words[0])

		# Get FoV
		ang_range = self.params["laser"][laser_id+"_fov"]
		if ang_range is None:
                        ang_range = float( 180 )
		else:
			ang_range = radians( float(ang_range) )

		# Get angular resolution
		ang_res = self.params["laser"][laser_id+"_resolution"]
		if ang_res is None:
                        ang_res = (radians(ang_range) / (num_range_readings - 0.0))
		else:
			ang_res = radians( float(ang_res) )
			
		# Get max reading
		max_reading = self.params["robot"][laser_id+"_max"]
		if max_reading is None:
			max_reading = 20.
		else:
			max_reading = float(max_reading)

		# init laser msg
		ang_min = radians( - ang_range / 2. )
		ang_max = radians( + ang_range / 2. ) - ang_res

		self.laser_msg.angle_min 	   = ang_min
		self.laser_msg.angle_max 	   = ang_max
		self.laser_msg.angle_increment = ang_res
		
		self.laser_msg.range_min = 0
		self.laser_msg.range_max = max_reading
			
		self.laser_msg.ranges = ranges		
		self.laser_msg.header.stamp = rospy.Time( float(words[last_range_reading+7]) )
		
		#TODO : x y theta odom_x odom_y odom_theta
		#       find out what they are and deal with it

		robot_link = self.links["ROBOT"]
		odom_link  = self.links["ROBOTODOM"]

		# tf needs to be publish a little bit in the future
		self.tf_odom_robot_msg.header.stamp = self.laser_msg.header.stamp + rospy.Duration(0.01)

		position = Point(float(words[last_range_reading+4]), float(words[last_range_reading+5]), 0.0)
		quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, float(words[last_range_reading+6]))

		self.tf_odom_robot_msg.header.frame_id 	  	 = odom_link
		self.tf_odom_robot_msg.child_frame_id 		 = robot_link
		self.tf_odom_robot_msg.transform.translation = position
		self.tf_odom_robot_msg.transform.rotation.x  = quaternion[0]
		self.tf_odom_robot_msg.transform.rotation.y  = quaternion[1]
		self.tf_odom_robot_msg.transform.rotation.z  = quaternion[2]
		self.tf_odom_robot_msg.transform.rotation.w  = quaternion[3]

		self.tf2_msg.transforms.append(self.tf_odom_robot_msg)
		
	def fillUpRobotLaserMessage(self, words):

		laser_link = self.links[words[0]]
		robot_link = self.links["ROBOT"]
		odom_link  = self.links["ROBOTODOM"]

		self.laser_msg.header.frame_id = robot_link #laser_link
		
		self.laser_msg.angle_increment = float(words[4])
		self.laser_msg.angle_min = float(words[2])
		self.laser_msg.angle_max = float(words[2])+float(words[3])

		self.laser_msg.range_min = 0.
		self.laser_msg.range_max = float(words[5])

		ranges = []
		num_range_readings = int(words[8])
		last_range_reading = num_range_readings + 8
		
		for word in words[9:last_range_reading+1]:
			ranges.append( float( word ))

		self.laser_msg.ranges = ranges

		# min-max angle fitting, Karto need
		factor_angle_fitting = self.laser_msg.angle_increment / float(2)
		while (round((self.laser_msg.angle_max - self.laser_msg.angle_min)/self.laser_msg.angle_increment) + 1) != num_range_readings:
			if (round((self.laser_msg.angle_max - self.laser_msg.angle_min)/self.laser_msg.angle_increment) + 1) > num_range_readings:
				self.laser_msg.angle_min = self.laser_msg.angle_min + factor_angle_fitting
			else
				self.laser_msg.angle_max = self.laser_msg.angle_max - factor_angle_fitting

			factor_angle_fitting = factor_angle_fitting / float(2)

		self.laser_msg.header.stamp = rospy.Time( float(words[last_range_reading+13]) )

		position = Point(float(words[last_range_reading+2]), float(words[last_range_reading+3]), 0.0)
		quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, float(words[last_range_reading+4]))

		self.tf_laser_robot_msg.header.stamp = self.laser_msg.header.stamp

		self.tf_laser_robot_msg.header.frame_id 	  = robot_link
		self.tf_laser_robot_msg.child_frame_id 		  = laser_link
		self.tf_laser_robot_msg.transform.translation = position
		self.tf_laser_robot_msg.transform.rotation.x  = quaternion[0]
		self.tf_laser_robot_msg.transform.rotation.y  = quaternion[1]
		self.tf_laser_robot_msg.transform.rotation.z  = quaternion[2]
		self.tf_laser_robot_msg.transform.rotation.w  = quaternion[3]
		
		# This transform is actually odom->laser_link
		# need to compute base_link->laser_link
		#self.tf2_msg.transforms.append(self.tf_laser_robot_msg)
		
		position = Point(float(words[last_range_reading+5]), float(words[last_range_reading+6]), 0.0)
		quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, float(words[last_range_reading+7]))

		# tf needs to be publish a little bit in the future
		self.tf_odom_robot_msg.header.stamp = self.laser_msg.header.stamp + rospy.Duration(0.01)

		self.tf_odom_robot_msg.header.frame_id 	  	 = odom_link
		self.tf_odom_robot_msg.child_frame_id 		 = robot_link
		self.tf_odom_robot_msg.transform.translation = position
		self.tf_odom_robot_msg.transform.rotation.x  = quaternion[0]
		self.tf_odom_robot_msg.transform.rotation.y  = quaternion[1]
		self.tf_odom_robot_msg.transform.rotation.z  = quaternion[2]
		self.tf_odom_robot_msg.transform.rotation.w  = quaternion[3]

		self.tf2_msg.transforms.append(self.tf_odom_robot_msg)
		
		self.pose_msg.header.stamp = self.laser_msg.header.stamp
		
		self.pose_msg.pose.pose.position 	  = position
		self.pose_msg.pose.pose.orientation.x = quaternion[0]
		self.pose_msg.pose.pose.orientation.y = quaternion[1]
		self.pose_msg.pose.pose.orientation.z = quaternion[2]
		self.pose_msg.pose.pose.orientation.w = quaternion[3]
		
		self.pose_msg.header.frame_id = odom_link
		self.pose_msg.child_frame_id  = robot_link
		
		#TODO : laser_tv laser_rv forward_safety_dist side_safty_dist
		#       find out what they are and deal with it
		
	def fillUpOdomMessage(self, words):

		self.pose_msg.header.stamp = rospy.Time( float(words[7]) )

		position = Point(float(words[1]), float(words[2]), 0.0)
		self.pose_msg.pose.pose.position = position
				
		quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, float(words[3]))
		self.pose_msg.pose.pose.orientation.x = quaternion[0]
		self.pose_msg.pose.pose.orientation.y = quaternion[1]
		self.pose_msg.pose.pose.orientation.z = quaternion[2]
		self.pose_msg.pose.pose.orientation.w = quaternion[3]
		
		odom_link  = self.links["ODOM"]
		robot_link = self.links["ROBOT"]
		
		self.pose_msg.header.frame_id = odom_link
		self.pose_msg.child_frame_id  = robot_link
		
		# TODO : fill up covariance & twist ?
		#self.pose_msg.twist.linear  = float(words[4])
		#self.pose_msg.twist.angular = float(words[5])

		if not self.publish_corrected:
			self.tf_odom_robot_msg.header.stamp 		 = self.pose_msg.header.stamp
			self.tf_odom_robot_msg.header.frame_id 	  	 = odom_link
			self.tf_odom_robot_msg.child_frame_id 		 = robot_link
			self.tf_odom_robot_msg.transform.translation = position
			self.tf_odom_robot_msg.transform.rotation.x  = quaternion[0]
			self.tf_odom_robot_msg.transform.rotation.y  = quaternion[1]
			self.tf_odom_robot_msg.transform.rotation.z  = quaternion[2]
			self.tf_odom_robot_msg.transform.rotation.w  = quaternion[3]

			self.tf2_msg.transforms.append(self.tf_odom_robot_msg)
		
	def fillUpTruePoseMessage(self, words):
		
		self.true_pose_msg.header.stamp = self.stamp

		position = Point(float(words[1]), float(words[2]), 0.0)
		self.true_pose_msg.pose.pose.position = position
				
		quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, radians(float(words[3])))
		self.true_pose_msg.pose.pose.orientation.x = quaternion[0]
		self.true_pose_msg.pose.pose.orientation.y = quaternion[1]
		self.true_pose_msg.pose.pose.orientation.z = quaternion[2]
		self.true_pose_msg.pose.pose.orientation.w = quaternion[3]
		
		odom_link  = self.links[words[0]]
		robot_link = self.links["ROBOT"]
		
		self.pose_msg.header.frame_id = odom_link
		
		# TODO : fill up covariance & twist ?
		#self.pose_msg.child_frame_id  = ?_link
		#self.pose_msg.twist.linear  = float(words[4])
		#self.pose_msg.twist.angular = float(words[5])
				
		self.tf_odom_robot_msg.header.stamp 		 = self.true_pose_msg.header.stamp
		self.tf_odom_robot_msg.header.frame_id 	  	 = odom_link
		self.tf_odom_robot_msg.child_frame_id 		 = robot_link
		self.tf_odom_robot_msg.transform.translation = position
		self.tf_odom_robot_msg.transform.rotation.x  = quaternion[0]
		self.tf_odom_robot_msg.transform.rotation.y  = quaternion[1]
		self.tf_odom_robot_msg.transform.rotation.z  = quaternion[2]
		self.tf_odom_robot_msg.transform.rotation.w  = quaternion[3]

		# TODO : search in self.tf2_msg if this links has been apend
		# if yes update, if no apend
		#self.tf2_msg.transforms[0] = self.tf_odom_robot_msg
		self.tf2_msg.transforms.append(self.tf_odom_robot_msg)
		
	def param(self, words):
		
		i_s = words[1].index("_")
		prefix = words[1][:i_s]
		actual_param = words[1][i_s+1:]
				
		try:
			dic = self.params[prefix]
		except KeyError:
			error_mess = "Unknown param prefix  '" + prefix + "'  param  '" + actual_param + "'  ignored !"
			rospy.logerr(error_mess)
			return
			
		try:
			dic[actual_param] = words[2]
			rospy.loginfo("Param  '%s_%s'  has value : %s", prefix, actual_param, words[2])
		except KeyError:
			rospy.logerr("Unknown param %s !", actual_param)
			return
	
	def increment_stamp(self):
		self.stamp = self.stamp + rospy.Duration(1./self.rate)
				
	def close_bag(self):
		self.bag.close()
		
	def print_help(self):
		rospy.loginfo("\nsimple usage: rosrun carmen_publisher carmen2rosbag.py -i path/to/carmen.log -o path/to/write/rosbag.bag")

if __name__ == '__main__':
	
	rospy.init_node('carmen2rosbag', anonymous=True)
    
	converter = carmen2rosbag()
	
	if len(sys.argv) < 3:
		converter.print_help()
		sys.exit()

	try:
		opts, args = getopt.getopt(sys.argv[1:],"hi:o:",["ifile=","ofile="])
	except getopt.GetoptError:
		converter.print_help()
		sys.exit()
		
	for opt, arg in opts:
		if opt in ('-h', "--help"):
			converter.print_help()
			sys.exit()
		elif opt in ("-i", "--ifile"):
			inputfile = arg
		elif opt in ("-o", "--ofile"):
			outputfile = arg
		
	converter.convert(inputfile, outputfile)
	
	converter.close_bag()
	
	rospy.loginfo("Job's done.")
