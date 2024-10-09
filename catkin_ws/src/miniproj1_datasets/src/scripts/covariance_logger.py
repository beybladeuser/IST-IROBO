#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
import os
import numpy as np
import sys

class CovarianceLogger:
	def __init__(self, topic_name = '/frame_path'):
		self.covariance = np.array([])
		subbed = False
		while not subbed:
			rospy.loginfo("Waiting for topic")
			try:
				self.covariance_sub = rospy.Subscriber(topic_name, Odometry, self.pose_callback)
				subbed = True
			except:
				pass
	def pose_callback(self, data):
		self.covariance = np.array(data.pose.covariance)

	def log(self):
		if len(self.covariance) != 0:
			str_cov = [str(x) for x in self.covariance]
			rospy.loginfo("Cov: " + " ".join(str_cov))

if __name__ == '__main__':
	rospy.init_node('covariance_logger')
	topic_name = rospy.get_param('~topic_name', '/odometry/filtered')  # The reference frame to track against
	rate_num = rospy.get_param('~rate', 10)  # The reference frame to track against

	if rospy.rostime.is_wallclock():
		rospy.logfatal('You should be using simulated time: rosparam set use_sim_time true')
		sys.exit(1)
	
	rospy.loginfo('Waiting for clock')
	rospy.sleep(0.00001)

	covariance_logger = CovarianceLogger(topic_name)

	rate = rospy.Rate(rate_num)  # Adjust the rate as necessary
	try:
		while not rospy.is_shutdown():
			covariance_logger.log()
			rate.sleep()
	except rospy.exceptions.ROSInterruptException:
		pass