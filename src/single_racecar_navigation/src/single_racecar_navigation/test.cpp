/*
 * @Author: Gong, Ke gk.kingj@gmail.com
 * @Date: 2023-02-14 14:47:38
 * @LastEditors: Gong, Ke gk.kingj@gmail.com
 * @LastEditTime: 2023-02-14 20:57:22
 * @FilePath: /src/single_racecar_navigation/src/single_racecar_navigation/test.cpp
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#include <fstream>
#include <string>
#include <vector>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <sys/wait.h>
#include <tf2/utils.h>
#include <tf2/convert.h>
#include <sys/shm.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <multi_turtlebot3_navigation/utils.h>

#include <opencv2/opencv.hpp>
#include <Astar/Astar.h>
#include <Astar/OccMapTransform.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

OccupancyGridParam OccGridParam;
pathplanning::Astar astar;
pathplanning::AstarConfig config;
nav_msgs::OccupancyGrid OccGridMask;
double InflateRadius;
cv::Point2d src_point;
cv::Point2d tar_point;

void MapCallback(const nav_msgs::OccupancyGrid& msg)
{
    // Get parameter
    OccGridParam.GetOccupancyGridParam(msg);

    // Get map
    int height = OccGridParam.height;
    int width = OccGridParam.width;
    int OccProb;
    cv::Mat Map(height, width, CV_8UC1);
    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {
            OccProb = msg.data[i * width + j];
            OccProb = (OccProb < 0) ? 100 : OccProb; // set Unknown to 0
            // The origin of the OccGrid is on the bottom left corner of the map
            Map.at<uchar>(height-i-1, j) = 255 - round(OccProb * 255.0 / 100.0);
        }
    }

    // Initial Astar
    cv::Mat Mask;
    config.InflateRadius = round(InflateRadius / OccGridParam.resolution);
    astar.InitAstar(Map, Mask, config);

    // Publish Mask
    OccGridMask.header.stamp = ros::Time::now();
    OccGridMask.header.frame_id = "/racecar_0/move_base/local_costmap/costmap";
    OccGridMask.info = msg.info;
    OccGridMask.data.clear();
    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {
            OccProb = Mask.at<uchar>(height-i-1, j) * 255;
            OccGridMask.data.push_back(OccProb);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;

    std::string robotName = "racecar_0";

    // astar
    ros::Subscriber map_sub = nh.subscribe("map", 10, MapCallback);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("astar_path", 10);
    ros::spinOnce();
    cv::Point init_p;
    cv::Point target_p;
    

    std::vector<cv::Point> PathList;
    nav_msgs::Path path;
    bool is_start = false;
    ros::Rate loop_rate(10);
    // while(ros::ok()){
    //     if(is_start){
    //         src_point = cv::Point2d(5.5, 45.0);
    //         OccGridParam.Map2ImageTransform(src_point, init_p);
    //         tar_point = cv::Point2d(6.50315332413, 40.2828292847);
    //         OccGridParam.Map2ImageTransform(tar_point, target_p);
    //         ROS_INFO("Astar Planning...");
    //         astar.PathPlanning(init_p,target_p,PathList);
    //         ROS_INFO("Astar Plann Successfully");
    //         if(!PathList.empty()) {
    //             path.header.stamp = ros::Time::now();
    //             path.header.frame_id = "map";
    //             path.poses.clear();
    //             for(int i=0;i<PathList.size();i++)
    //             {
    //                 cv::Point2d dst_point;
    //                 OccGridParam.Image2MapTransform(PathList[i], dst_point);

    //                 geometry_msgs::PoseStamped pose_stamped;
    //                 pose_stamped.header.stamp = ros::Time::now();
    //                 pose_stamped.header.frame_id = "map";
    //                 pose_stamped.pose.position.x = dst_point.x;
    //                 pose_stamped.pose.position.y = dst_point.y;
    //                 pose_stamped.pose.position.z = 0;
    //                 path.poses.push_back(pose_stamped);
    //             }
    //             path_pub.publish(path);
    //             double end_time = ros::Time::now().toSec();

    //             // ROS_INFO("Find a valid path successfully! Use %f s", end_time - start_time);
    //         } else{
    //             ROS_INFO("path empty!");
    //         }
    //     }
    //     is_start = true;
    //     loop_rate.sleep();
    //     ros::spinOnce();
    // }
    

    

    geometry_msgs::Pose target;
    target.position.x = 6.50315332413;
    target.position.y = 40.2828292847;
    // target.position.x = 1.72806;
    // target.position.y = 28.8451;
    target.position.z = 0.0;
    target.orientation.x = 0.0;
    target.orientation.y = 0.0;
    // target.orientation.z = 1.0;
    // target.orientation.w = 0.0;
    target.orientation.z = 1.0;
    target.orientation.w = 0.0;

    // execute navigation
    bool succeed = false;
    MoveBaseClient ac("/" + robotName + "/move_base", true);
    // wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    std::cout << "move base client: " << "/"+robotName+"/move_base" << std::endl;
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose = target;
    std::cout << "sending goal: " << goal.target_pose.pose.position.x << " "<<goal.target_pose.pose.position.y << std::endl;
    
    ac.sendGoal(goal);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        succeed = true;
    }
    if(!succeed){
        std::cout << "can't reach the target position, navigate failed!" << std::endl;
        ros::shutdown();
        return 0;
    }
}