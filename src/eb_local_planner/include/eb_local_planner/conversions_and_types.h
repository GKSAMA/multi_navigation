//
// Created by gk on 2022/2/21.
//

#ifndef EB_LOCAL_PLANNER_CONVERSIONS_AND_TYPES_H
#define EB_LOCAL_PLANNER_CONVERSIONS_AND_TYPES_H

#include <ros/ros.h>

// msgs
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs//Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/TransformStamped.h"

// transforms
#include "angles/angles.h"
#include "tf2/convert.h"
#include "tf2/transform_datatypes.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"

// costmap & geometry
#include "costmap_2d/costmap_2d_ros.h"

namespace eb_local_planner{

    struct Bubble{
        geometry_msgs::PoseStamped center;
        double expansion;
    };

    enum AddAtPosition {add_front, add_back};

    void PoseToPose2D(const geometry_msgs::Pose pose, geometry_msgs::Pose2D &pose2D);

    void Pose2DToPose(const geometry_msgs::Pose2D pose2D, geometry_msgs::Pose &pose);

    bool transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                             costmap_2d::Costmap2DROS& costmap, const std::string& global_frame,
                             std::vector<geometry_msgs::PoseStamped>& transformed_plan, std::vector<int>& start_end_counts_from_end);
    double  getCircumscribedRadius(costmap_2d::Costmap2DROS& costmap);
}

#endif //EB_LOCAL_PLANNER_CONVERSIONS_AND_TYPES_H
