
#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import math

class PositionErrorPublisher:
    def __init__(self):
        # Initialize the node
        rospy.init_node('position_error_publisher')

        # Subscribe to the ground truth and estimated pose topics
        self.groundtruth_sub = rospy.Subscriber('/tf', tf.msg.tfMessage, self.groundtruth_callback)
        self.estimated_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.estimated_callback)

        # Publisher for error
        self.error_pub = rospy.Publisher('/position_error', Odometry, queue_size=10)

        # Initialize variables
        self.groundtruth_pose = None
        self.estimated_pose = None

    def groundtruth_callback(self, data):
        # Extract ground truth pose from the TF message (assuming ground truth is published in mocap->base_link)
        for transform in data.transforms:
            if transform.child_frame_id == "base_link" and transform.header.frame_id == "mocap":
                self.groundtruth_pose = transform.transform.translation

    def estimated_callback(self, data):
        # Extract estimated pose from Odometry message
        self.estimated_pose = data.pose.pose.position
        # Calculate the error if ground truth is available
        if self.groundtruth_pose:
            self.calculate_error()

    def calculate_error(self):
        # Compute positional error (Euclidean distance)
        error_x = self.groundtruth_pose.x - self.estimated_pose.x
        error_y = self.groundtruth_pose.y - self.estimated_pose.y
        error_z = self.groundtruth_pose.z - self.estimated_pose.z

        error = math.sqrt(error_x**2 + error_y**2 + error_z**2)
        rospy.loginfo("Position Error: %f", error)

        # Publish error (you can also publish it as an Odometry or custom message if needed)
        error_msg = Odometry()
        error_msg.header.stamp = rospy.Time.now()
        error_msg.header.frame_id = "map"
        error_msg.pose.pose.position.x = error
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        PositionErrorPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass