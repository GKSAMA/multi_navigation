#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Twist.h"
#include "boost/function.hpp"
#include "multi_turtlebot3_navigation/RunState.h"

void ob0cb(const ros::TimerEvent& event, int* flag,ros::Publisher pub){
    // ROS_INFO("obstacle0 direction change ...flag is:%d", *flag);
    geometry_msgs::Twist twist;
    if(*flag == 0)
        *flag = 1;
    else
        *flag = 0;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    pub.publish(twist);
}

void ob1cb(const ros::TimerEvent& event, int* flag,ros::Publisher pub){
    // ROS_INFO("obstacle1 direction change ...flag is:%d", *flag);
    geometry_msgs::Twist twist;
    if(*flag == 0)
        *flag = 1;
    else
        *flag = 0;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    pub.publish(twist);
}

int main(int argc, char **argv){
    ros::init(argc, argv,"move_obstacle");
    ros::NodeHandle nh;
    ros::Publisher ob0_pub = nh.advertise<geometry_msgs::Twist>("/ob0/cmd_vel", 1);
    ros::Publisher ob1_pub = nh.advertise<geometry_msgs::Twist>("/ob1/cmd_vel", 1);
    int forward0 = 1, forward1 = 1;
    geometry_msgs::Twist ob0_tw0,ob0_tw1;
    geometry_msgs::Twist ob1_tw0,ob1_tw1;
    ob0_tw0.linear.x = 0.3;
    ob0_tw0.linear.y = 0.0;
    ob0_tw0.linear.z = 0.0;
    ob0_tw0.angular.x = 0.0;
    ob0_tw0.angular.y = 0.0;
    ob0_tw0.angular.z = 0.0;

    ob0_tw1.linear.x = -0.3;
    ob0_tw1.linear.y = 0.0;
    ob0_tw1.linear.z = 0.0;
    ob0_tw1.angular.x = 0.0;
    ob0_tw1.angular.y = 0.0;
    ob0_tw1.angular.z = 0.0;

    ob1_tw0.linear.x = 0.3;
    ob1_tw0.linear.y = 0.0;
    ob1_tw0.linear.z = 0.0;
    ob1_tw0.angular.x = 0.0;
    ob1_tw0.angular.y = 0.0;
    ob1_tw0.angular.z = 0.0;

    ob1_tw1.linear.x = -0.3;
    ob1_tw1.linear.y = 0.0;
    ob1_tw1.linear.z = 0.0;
    ob1_tw1.angular.x = 0.0;
    ob1_tw1.angular.y = 0.0;
    ob1_tw1.angular.z = 0.0;
    ros::Timer timer0 = nh.createTimer(ros::Duration(15),boost::bind(&ob0cb,_1,&forward0,ob0_pub));
    ros::Timer timer1 = nh.createTimer(ros::Duration(15),boost::bind(&ob1cb,_1,&forward1,ob1_pub));

    while(ros::ok()){
        // ROS_INFO("Obstacle0 moving ... ");
        // ROS_INFO("flag0:%d",forward0);
        if(forward0)
            ob0_pub.publish(ob0_tw0);
        else
            ob0_pub.publish(ob0_tw1);
        // ROS_INFO("Obstacle1 moving ... ");
        // ROS_INFO("flag1:%d",forward1);
        if(forward1)
            ob1_pub.publish(ob1_tw0);
        else
            ob1_pub.publish(ob1_tw1);
        ros::spinOnce();
    }
    
    return 0;
}