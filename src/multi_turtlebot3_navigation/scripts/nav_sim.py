#!/usr/bin/env python
import rospy
import std_msgs.msg
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
 
import time
import threading
import argparse
 
def thread_job():
    rospy.spin()
 
def callback(data, args):
    speed = data.linear.x
    turn = data.angular.z
    # pub = rospy.Publisher("/racecar_0/ackermann_cmd_mux/output", AckermannDriveStamped, queue_size=1)
 
    msg = AckermannDriveStamped();  
    msg.header.stamp = rospy.Time.now();
    msg.header.frame_id = args[0] + "/base_link";

    msg.drive.speed = speed;
    msg.drive.acceleration = 1;
    msg.drive.jerk = 1;
    msg.drive.steering_angle = turn 
    msg.drive.steering_angle_velocity = 1

    # rosinfo("speed" + speed)
    args[1].publish(msg)
    # pub.publish(msg)

def SubscribeAndPublish(namespace):
    rospy.init_node('nav_sim', anonymous=True)
    pub = rospy.Publisher(namespace+"/ackermann_cmd_mux/output", AckermannDriveStamped, queue_size=1)
    rospy.Subscriber(namespace+'/cmd_vel', Twist, callback,(namespace, pub),queue_size=1,buff_size=52428800)
    # rospy.Subscriber(namespace+'/cmd_vel', Twist, callback,queue_size=1,buff_size=52428800)
    #rospy.Subscriber('cmd_vel', Twist, callback,queue_size=1,buff_size=52428800)
    rospy.spin()

def parser_args(args=None):
    parser = argparse.ArgumentParser(description='namespace')
    parser.add_argument('--namespace', metavar='ns', default='/racecar_0')
    return parser.parse_args(args=args)

if __name__ == '__main__':
    args=parser_args(rospy.myargv()[1:])
    try:
        SubscribeAndPublish(args.namespace)
    except rospy.ROSInterruptException:
        pass


########################

