import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class FramePathPublisher:
    def __init__(self, frame_id, ref_frame="world", topic_name = '/frame_path'):
        self.frame_id = frame_id
        self.ref_frame = ref_frame
        self.path_pub = rospy.Publisher(topic_name, Path, queue_size=10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.ref_frame

        self.listener = tf.TransformListener()

    def publish_path(self):
        try:
            # Lookup the transform between the reference frame and the frame of interest
            (trans, rot) = self.listener.lookupTransform(self.ref_frame, self.frame_id, rospy.Time(0))

            # Create a PoseStamped from the transform
            pose = PoseStamped()
            pose.header.frame_id = self.ref_frame
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = trans[0]
            pose.pose.position.y = trans[1]
            pose.pose.position.z = trans[2]
            pose.pose.orientation.x = rot[0]
            pose.pose.orientation.y = rot[1]
            pose.pose.orientation.z = rot[2]
            pose.pose.orientation.w = rot[3]

            # Append the new pose to the path
            self.path_msg.poses.append(pose)

            # Publish the path
            self.path_pub.publish(self.path_msg)
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

if __name__ == '__main__':
    rospy.init_node('frame_path_publisher')
    frame_id = rospy.get_param('~frame_id', 'base_scan')  # The frame you want to track
    ref_frame = rospy.get_param('~ref_frame', 'map')  # The reference frame to track against
    topic_name = rospy.get_param('~topic_name', '/frame_path')  # The reference frame to track against

    frame_path_publisher = FramePathPublisher(frame_id, ref_frame, topic_name)
    
    rate = rospy.Rate(10)  # Adjust the rate as necessary
    while not rospy.is_shutdown():
        frame_path_publisher.publish_path()
        rate.sleep()