/*
 * @Author: Gong, Ke gk.kingj@gmail.com
 * @Date: 2022-11-21 13:49:09
 * @LastEditors: Gong, Ke gk.kingj@gmail.com
 * @LastEditTime: 2023-05-02 14:00:32
 * @FilePath: /src/single_racecar_navigation/src/draw_line.cpp
 * @Description: 
 * 
 * Copyright (c) 2022 by Gong, Ke gk.kingj@gmail.com, All Rights Reserved. 
 */
#include <ros/ros.h>
#include "tf/transform_listener.h"
#include "tf2/convert.h"
#include "tf2/transform_datatypes.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/Marker.h"
#include "angles/angles.h"
#include "sys/shm.h"
#include "fstream"
#include "iostream"
#include <multi_turtlebot3_navigation/utils.h>
#include <boost/bind.hpp>

void callback(const geometry_msgs::PoseStamped::ConstPtr &msg, geometry_msgs::PoseStamped *rearPose)
{
    rearPose->pose.position.x = msg->pose.position.x;
    rearPose->pose.position.y = msg->pose.position.y;
    rearPose->pose.position.z = msg->pose.position.z;
    rearPose->pose.orientation.x = msg->pose.orientation.x;
    rearPose->pose.orientation.y = msg->pose.orientation.y;
    rearPose->pose.orientation.z = msg->pose.orientation.z;
    rearPose->pose.orientation.w = msg->pose.orientation.w;
}

void rearLeftCB(const geometry_msgs::Point::ConstPtr &msg, geometry_msgs::Point *rearLeftPoint)
{
    rearLeftPoint->x = msg->x;
    rearLeftPoint->y = msg->y;
    rearLeftPoint->z = msg->z;
    // std::cout << " x = " << msg->x << " y = " << msg->y << " z = " << msg->z << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "single_show_path");
    ros::NodeHandle n;
    ros::Publisher racecar_marker_pub = n.advertise<visualization_msgs::Marker>("path_marker",10);
    geometry_msgs::PoseStamped rearPose;
    std::string ns = "smart_0";
    if(argc == 2){
        // std::cout << argv[0] << " " <<argv[1] << std::endl;
        ns = argv[1];
    }
    ros::Subscriber rear_sub = n.subscribe<geometry_msgs::PoseStamped>("/" + ns + "/rear_pose", 10, boost::bind(&callback, _1, &rearPose));
    geometry_msgs::Point rearLeft;
    // ros::Subscriber rearLeft_sub = n.subscribe<geometry_msgs::Point>("/racecar_0/rear_left", 10, boost::bind(&rearLeftCB, _1, &rearLeft));
    ros::Rate r(1);
    tf::TransformListener listener;
    
    visualization_msgs::Marker line_strip;
    ros::Time start, end;
    line_strip.header.frame_id = "/map";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "single_show_path";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 1;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.05;
    line_strip.color.a = 1.0;
    line_strip.color.g = 0.5;
    line_strip.color.r = 1.0;

    int cnt(0);
    geometry_msgs::Point p;
    geometry_msgs::Pose pose;
    tf::StampedTransform transform;
    ros::Duration(1.0).sleep();
    try {
        listener.lookupTransform("/map", ns + "/rear_axis_center", ros::Time(0), transform);
    }
    catch(tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    MultiNavUtil::setPoint(p, transform);
    MultiNavUtil::setPose(pose, transform);

    while (ros::ok())
    {
        try {
            listener.lookupTransform("/map", ns + "/rear_axis_center", ros::Time(0), transform);
        }
        catch(tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        MultiNavUtil::setPoint(p, transform);
        // std::cout << rearPose.pose.position.x << "  " << rearPose.pose.position.y << std::endl;
        // std::cout << rearLeft.x << "  " << rearLeft.y << std::endl;
        // std::cout << p.x << "  " << p.y << std::endl;
        if(cnt > 1) {
            // line_strip.points.push_back(p);
            // line_strip.points.push_back(rearPose.pose.position);
            line_strip.points.push_back(p);
        } else {
            ++cnt;
        }
        racecar_marker_pub.publish(line_strip);
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
    
}