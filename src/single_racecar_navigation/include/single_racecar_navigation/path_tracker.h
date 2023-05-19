/*
 * @Author: Gong, Ke gk.kingj@gmail.com
 * @Date: 2022-12-14 15:38:07
 * @LastEditors: Gong, Ke gk.kingj@gmail.com
 * @LastEditTime: 2023-04-24 10:30:59
 * @FilePath: /src/single_racecar_navigation/include/single_racecar_navigation/path_tracker.h
 * @Description: 
 * 
 * Copyright (c) 2022 by Gong, Ke gk.kingj@gmail.com, All Rights Reserved. 
 */
#ifndef PATH_TRACKER_H
#define PATH_TRACKER_H

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>
#include <tf2/convert.h>

class PathTracker{
public:
    PathTracker();
    ~PathTracker();

    geometry_msgs::Twist CalculateTwistCommand();
    

private:
    void pathCB(const nav_msgs::Path::ConstPtr& path);
    void odomCB(const geometry_msg::Odometry::ConstPtr& odom);
    nav_msgs::Path global_path_;
    ros::Subscriber global_path_sub_;
    ros::Publisher cmd_pub_;
    ros::Subscriber current_pose_sub_;
    geometry_msgs::Pose current_pose_;
    geometry_msgs::Pose target_pose_;
    
};

#endif