import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import miniporj1_utils as utils
import os
import pandas as pd

class CovarianceLogger:
	def __init__(self, output, topic_name = '/frame_path'):
		self.output = output
		self.covariance = pd.DataFrame()
		self.covariance_sub = rospy.Subscriber(topic_name, Odometry, self.pose_callback)

	def pose_callback(self, data):
		self.covariance = pd.concat([self.covariance, pd.DataFrame([data.pose.covariance]).rename_axis("id") ], ignore_index=True)

	def log(self):
		try:
			self.covariance.to_csv(self.output)		
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			pass

if __name__ == '__main__':
	rospy.init_node('covariance_logger')
	topic_name = rospy.get_param('~topic_name', '/odometry/filtered')  # The reference frame to track against
	rate_num = rospy.get_param('~rate', 10)  # The reference frame to track against
	output_filename = rospy.get_param('~output', './covariance')  # The reference frame to track against
	output_filename = utils.expand_filename(output_filename)
	output_filename = utils.split_extension_from_filename(output_filename)[0]
	output_index = utils.get_dup_file_index(output_filename)
	output_filename = f"${output_filename}_${output_index}.csv"
	utils.create_file(output_filename)

	covariance_logger = CovarianceLogger(output_filename, topic_name)

	rate = rospy.Rate(rate_num)  # Adjust the rate as necessary
	try:
		while not rospy.is_shutdown():
			covariance_logger.log()
			rate.sleep()
	except rospy.exceptions.ROSInterruptException:
		pass