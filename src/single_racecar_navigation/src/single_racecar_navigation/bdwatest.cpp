/*
 * @Author: Gong, Ke gk.kingj@gmail.com
 * @Date: 2023-02-14 14:47:38
 * @LastEditors: Gong, Ke gk.kingj@gmail.com
 * @LastEditTime: 2023-04-24 10:58:56
 * @FilePath: /src/single_racecar_navigation/src/single_racecar_navigation/bdwatest.cpp
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
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "visualization_msgs/Marker.h"

#include <multi_turtlebot3_navigation/utils.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bdwatest");
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("goal_marker", 1);
    std::string robotName = "smart_0";

    geometry_msgs::Pose target;
    tf2::Quaternion q;
    double yaw;
    // std::string pathToPackage = ros::package::getPath("single_racecar_navigation");
    std::string pathToPackage = "/home/gk/Documents/multi_turtlebot3_navigation/src/single_racecar_navigation";
    YAML::Node config = YAML::LoadFile(pathToPackage + "/src/single_racecar_navigation/testgoal.yaml");

    /**目标**/
    target.position.x = config["x"].as<float>();
    target.position.y = config["y"].as<float>();
    target.position.z = 0.0;
    yaw = config["yaw"].as<float>();
    q.setRPY(0, 0, yaw);
    tf2::convert(q, target.orientation);


    // /*目标点Marker*/
    visualization_msgs::Marker goal_marker;
    ros::Time start, end;
    goal_marker.header.frame_id = "/map";
    goal_marker.header.stamp = ros::Time::now();
    goal_marker.ns = "bdwatest";
    goal_marker.action = visualization_msgs::Marker::ADD;
    goal_marker.pose = target;
    goal_marker.id = 2;
    goal_marker.type = visualization_msgs::Marker::ARROW;
    goal_marker.scale.x = 1.0;
    goal_marker.scale.y = 1.0;
    goal_marker.scale.z = 1.0;
    goal_marker.color.a = 1.0;
    goal_marker.color.g = 0.0;
    goal_marker.color.r = 1.0;
    marker_pub.publish(goal_marker);

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