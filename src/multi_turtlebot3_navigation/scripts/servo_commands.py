#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped
import argparse

flag_move = 0

def set_throttle_steer(data, namespace):

    global flag_move

    pub_vel_left_rear_wheel = rospy.Publisher(namespace + '/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_rear_wheel = rospy.Publisher(namespace + '/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_left_front_wheel = rospy.Publisher(namespace + '/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_front_wheel = rospy.Publisher(namespace + '/right_front_wheel_velocity_controller/command', Float64, queue_size=1)

    pub_pos_left_steering_hinge = rospy.Publisher(namespace + '/left_steering_hinge_position_controller/command', Float64, queue_size=1)
    pub_pos_right_steering_hinge = rospy.Publisher(namespace + '/right_steering_hinge_position_controller/command', Float64, queue_size=1)

    throttle = data.drive.speed*13.95348
    steer = data.drive.steering_angle

    pub_vel_left_rear_wheel.publish(throttle)
    pub_vel_right_rear_wheel.publish(throttle)
    pub_vel_left_front_wheel.publish(throttle)
    pub_vel_right_front_wheel.publish(throttle)
    pub_pos_left_steering_hinge.publish(steer)
    pub_pos_right_steering_hinge.publish(steer)

def parser_args(args=None):
    parser = argparse.ArgumentParser(description='namespace')
    parser.add_argument('--namespace', metavar='ns', default='/racecar_2')
    return parser.parse_args(args=args)

def servo_commands():

    rospy.init_node('servo_commands', anonymous=True)
    args=parser_args(rospy.myargv()[1:])

    rospy.Subscriber(args.namespace + "/ackermann_cmd_mux/output", AckermannDriveStamped, set_throttle_steer, args.namespace)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass
