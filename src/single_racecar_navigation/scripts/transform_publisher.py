#!/usr/bin/env python  
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class transform_publisher():
	def __init__(self):
		rospy.init_node('transform_publisher')

		rospy.Subscriber('/smart_0/odom', Odometry, self.pose_cb, queue_size = 1)

		rospy.spin()

	def pose_cb(self, msg):
		pose = msg.pose.pose.position
		orientation = msg.pose.pose.orientation
		br = tf.TransformBroadcaster()
		br.sendTransform((pose.x, pose.y, pose.z),
							(orientation.x, orientation.y, orientation.z, orientation.w),
							rospy.Time.now(),
							'smart_0/base_link', 'world')


if __name__ == "__main__":
	try:
		transform_publisher()
	except:
		rospy.logwarn("cannot start transform publisher")