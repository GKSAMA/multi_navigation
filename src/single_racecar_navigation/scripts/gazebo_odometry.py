#!/usr/bin/env python

'''
This script makes Gazebo less fail by translating gazebo status messages to odometry data.
Since Gazebo also publishes data faster than normal odom data, this script caps the update to 20hz.
Winter Guerra
'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped, PoseStamped, Point
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import numpy as np
import math
import tf2_ros
import tf
import argparse

class OdometryNode:
    # Set publishers
    # pub_odom = rospy.Publisher('/odom', Odometry, queue_size=1)
    namespace = ""
    def __init__(self, ns):
        # init internals
        self.last_received_pose = Pose()
        self.last_received_twist = Twist()
        self.last_recieved_stamp = None
        self.namespace = ns
        self.rear_pose_pub = rospy.Publisher(self.namespace + '/rear_pose', PoseStamped, queue_size=1)
        self.pub_odom = rospy.Publisher(self.namespace+'/odom', Odometry, queue_size=1)
        self.rear_left_point = rospy.Publisher(self.namespace+'/rear_left', Point, queue_size=1)
        # Set the update rate
        rospy.Timer(rospy.Duration(0.01), self.timer_callback) # 20hz

        self.tf_pub = tf2_ros.TransformBroadcaster()

        # Set subscribers
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.sub_robot_pose_update)

    def sub_robot_pose_update(self, msg):
        # Find the index of the racecar
        try:
            arrayIndex = msg.name.index(self.namespace[1:]+'::base_footprint')
            leftRearIndex = msg.name.index('racecar_0::left_rear_wheel')
        except ValueError as e:
            # Wait for Gazebo to startup
            pass
        else:
            # Extract our current position information
            self.last_received_pose = msg.pose[arrayIndex]
            self.last_received_twist = msg.twist[arrayIndex]
            # print("cur x = ", self.last_received_pose.position.x, "cur y = ", self.last_received_pose.position.y)
            orientation = self.last_received_pose.orientation
            (_,_,yaw) = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
            rear_pose = PoseStamped()
            rear_pose.header.stamp = rospy.Time.now()
            rear_pose.header.frame_id = '/map'
            rear_pose.pose.position.x = self.last_received_pose.position.x - math.cos(yaw) * 0.167;
            rear_pose.pose.position.y = self.last_received_pose.position.y - math.sin(yaw) * 0.167;
            rear_pose.pose.orientation = self.last_received_pose.orientation
            self.rear_pose_pub.publish(rear_pose)
            rear_left_pose = Point()
            rear_left_pose.x = msg.pose[leftRearIndex].position.x;
            rear_left_pose.y = msg.pose[leftRearIndex].position.y;
            self.rear_left_point.publish(rear_left_pose)
        self.last_recieved_stamp = rospy.Time.now()

    def timer_callback(self, event):
        if self.last_recieved_stamp is None:
            return

        cmd = Odometry()
        cmd.header.stamp = self.last_recieved_stamp
        cmd.header.frame_id = self.namespace+'/odom'
        cmd.child_frame_id = self.namespace+'/base_footprint'
        cmd.pose.pose = self.last_received_pose
        cmd.twist.twist = self.last_received_twist
        cmd.pose.covariance =[1e-3, 0, 0, 0, 0, 0,
						0, 1e-3, 0, 0, 0, 0,
						0, 0, 1e6, 0, 0, 0,
						0, 0, 0, 1e6, 0, 0,
						0, 0, 0, 0, 1e6, 0,
						0, 0, 0, 0, 0, 1e3]

        cmd.twist.covariance = [1e-9, 0, 0, 0, 0, 0, 
                          0, 1e-3, 1e-9, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e-9]

        # print("cur x = ", cmd.pose.pose.position.x, "cur y = ", cmd.pose.pose.position.y)
        self.pub_odom.publish(cmd)

        tf = TransformStamped(
            header=Header(
                frame_id=cmd.header.frame_id,
                stamp=cmd.header.stamp
            ),
            child_frame_id=cmd.child_frame_id,
            transform=Transform(
                translation=cmd.pose.pose.position,
                rotation=cmd.pose.pose.orientation
            )
        )
        self.tf_pub.sendTransform(tf)

def parser_args(args=None): 
    parser = argparse.ArgumentParser(description='namespace')
    parser.add_argument('--namespace', metavar='ns', default='/racecar_0')
    parser.add_argument('--x', metavar='x', default='0.0')
    parser.add_argument('--y', metavar='y', default='0.0')
    parser.add_argument('--z', metavar='z', default='0.0')
    parser.add_argument('--th', metavar='th', default='0.0')
    return parser.parse_args(args=args)
if __name__ == '__main__':
    # namespace = rospy.get_param('~namespace','/racecar_0')
    # rospy.loginfo(namespace+"----------------------------------------------------")
    args=parser_args(rospy.myargv()[1:])
    rospy.init_node("gazebo_odometry_node")
    node = OdometryNode(args.namespace)
    rospy.spin()
