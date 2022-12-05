#include <Eigen/Core>
#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include "adwa_local_planner/adwa_utils.h"

void odomCB(const nav_msgs::Odometry::ConstPtr& odometry, nav_msgs::Odometry* odom){
    // double time = odometry->header.stamp.toSec();
    // double yaw = tf::getYaw(odometry->pose.pose.orientation);
    // geometry_msgs::Twist vel = odometry->twist.twist;
    // geometry_msgs::Pose pose = odometry->pose.pose;
    // geometry_msgs::Pose2D pose2D;
    // MultiNavUtil::PoseToPose2D(pose,pose2D);
    // odom = odometry;
    odom->twist = odometry->twist;
    odom->pose.pose.position.x = odometry->pose.pose.position.x;
    odom->pose.pose.position.y = odometry->pose.pose.position.y;
    odom->pose.pose.position.z = odometry->pose.pose.position.z;
    odom->pose.pose.orientation.x = odometry->pose.pose.orientation.x;
    odom->pose.pose.orientation.y = odometry->pose.pose.orientation.y;
    odom->pose.pose.orientation.z = odometry->pose.pose.orientation.z;
    odom->pose.pose.orientation.w = odometry->pose.pose.orientation.w;
    // std::cout << "x: " << odom->pose.pose.position.x << 
    //              " y: " << odom->pose.pose.position.y << 
    //              " z: " << odom->pose.pose.position.z << 
    //              " qx: " << odom->pose.pose.orientation.x << 
    //              " qy: " << odom->pose.pose.orientation.y << 
    //              " qz: " << odom->pose.pose.orientation.z << 
    //              " qw: " << odom->pose.pose.orientation.w << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"adwa_test");
    ros::NodeHandle nh;
    nav_msgs::Odometry *odom = new nav_msgs::Odometry();
    ros::Subscriber odomSub = nh.subscribe<nav_msgs::Odometry>("/racecar_0/odom",100,boost::bind(&odomCB, _1, odom));
    tf::TransformListener listener;
    tf::StampedTransform transform0;
    if(listener.canTransform("/map", "racecar_0/odom",ros::Time::now()))
        listener.lookupTransform("/map", "racecar_0/odom", ros::Time(0), transform0);
    Eigen::Matrix<float, 4, 1> R;
    R[0] = odom->pose.pose.orientation.x;
    R[1] = odom->pose.pose.orientation.y;
    R[2] = odom->pose.pose.orientation.z;
    R[3] = odom->pose.pose.orientation.w;
    Eigen::Matrix<float,3,1> T;
    T[0] = odom->pose.pose.position.x;
    T[1] = odom->pose.pose.position.y;
    T[2] = odom->pose.pose.position.z;
    Eigen::Matrix<float,3,1> p;
    p[0] = 1;
    p[1] = 0;
    p[2] = 0;
    Eigen::Matrix<float,3,1> result = adwa_utils::invertRigidTransform(T,R,p);
    while(ros::ok()){
        // std::cout << "x: " << odom->pose.pose.position.x << 
        //          " y: " << odom->pose.pose.position.y << 
        //          " z: " << odom->pose.pose.position.z << 
        //          " qx: " << odom->pose.pose.orientation.x << 
        //          " qy: " << odom->pose.pose.orientation.y << 
        //          " qz: " << odom->pose.pose.orientation.z << 
        //          " qw: " << odom->pose.pose.orientation.w << std::endl;
        R[0] = odom->pose.pose.orientation.x;
        R[1] = odom->pose.pose.orientation.y;
        R[2] = odom->pose.pose.orientation.z;
        R[3] = odom->pose.pose.orientation.w;
        T[0] = odom->pose.pose.position.x;
        T[1] = odom->pose.pose.position.y;
        T[2] = odom->pose.pose.position.z;
        result = adwa_utils::invertRigidTransform(T,R,p);
        std::cout << result << std::endl;
        ros::spinOnce();
        sleep(1);
    }
    // std::cout << result << std::endl;
    // ros::spin();
}