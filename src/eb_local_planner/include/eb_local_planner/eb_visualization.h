//
// Created by gk on 2022/2/22.
//

#ifndef EB_LOCAL_PLANNER_EB_VISUALIZATION_H
#define EB_LOCAL_PLANNER_EB_VISUALIZATION_H

#include <ros/ros.h>

// classes wich are part of this pkg
#include <eb_local_planner/conversions_and_types.h>
#include <eb_local_planner/EBPlannerConfig.h>

// msgs
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// transforms
#include <angles/angles.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>

// Eigen library for geometric operation
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace eb_local_planner{

    class EBVisualization{
    public:
        enum Color {blue, red, green};

        EBVisualization();
        EBVisualization(ros::NodeHandle& pn, costmap_2d::Costmap2DROS* costmap_ros);
        ~EBVisualization();

        void initialize(ros::NodeHandle& pn, costmap_2d::Costmap2DROS* costmap_ros);
        void reconfigure(EBPlannerConfig& config);
        void publishBand(std::string marker_name_space, std::vector<Bubble> band);
        void publishBubble(std::string marker_name_space, int marker_id, Bubble bubble);
        void publishBubble(std::string marker_name_space, int marker_id, Color marker_color, Bubble bubble);
        void publishForceList(std::string marker_name_space, std::vector<geometry_msgs::WrenchStamped> forces, std::vector<Bubble> band);
        void publishForce(std::string marker_name_space, int id, Color marker_color, geometry_msgs::WrenchStamped force, Bubble bubble);

    private:
        costmap_2d::Costmap2DROS* costmap_ros_;
        ros::Publisher bubble_pub_;
        ros::Publisher one_bubble_pub_;
        bool initialized_;
        double marker_lifetime_;

        void bubbleToMarker(Bubble bubble, visualization_msgs::Marker& marker,std::string marker_name_space, int marker_id, Color marker_color);
        void bubbleHeadingToMarker(Bubble bubble, visualization_msgs::Marker marker, std::string marker_name_space, int marker_id, Color marker_color);
        void forceToMarker(geometry_msgs::WrenchStamped wrench, geometry_msgs::Pose wrench_origin, visualization_msgs::Marker& marker,std::string marker_name_space, int marker_id, Color marker_color);
    };
};

#endif //EB_LOCAL_PLANNER_EB_VISUALIZATION_H
