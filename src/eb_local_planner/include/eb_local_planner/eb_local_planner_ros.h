//
// Created by gk on 2022/2/25.
//

#ifndef EB_LOCAL_PLANNER_EB_LOCAL_PLANNER_ROS_H
#define EB_LOCAL_PLANNER_EB_LOCAL_PLANNER_ROS_H
#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <dynamic_reconfigure/server.h>
#include "eb_local_planner/eb_local_planner.h"
#include "eb_local_planner/conversions_and_types.h"
#include "eb_visualization.h"
#include <eb_local_planner/eb_trajectory_controller.h>
#include <eb_local_planner/EBPlannerConfig.h>
#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <angles/angles.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

namespace eb_local_planner{
    class EBPlannerROS : public nav_core::BaseLocalPlanner{
    public:
        EBPlannerROS();
        EBPlannerROS(std::string name,tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
        ~EBPlannerROS();

        void initialize(std::string name,tf2_ros::Buffer* tf,costmap_2d::Costmap2DROS* costmap_map);
        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
        bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
        bool isGoalReached();
    private:
        void reconfigureCallback(EBPlannerConfig& config,uint32_t level);
        typedef dynamic_reconfigure::Server<eb_local_planner::EBPlannerConfig> drs;
        boost::shared_ptr<drs> drs_;
        costmap_2d::Costmap2DROS* costmap_ros_;
        tf2_ros::Buffer* tf_;
        double yaw_goal_tolerance_, xy_goal_tolerance_;
        double rot_stopped_vel_, trans_stopped_vel_;
        ros::Publisher g_plan_pub_;
        ros::Publisher l_plan_pub_;
        ros::Subscriber odom_sub_;
        nav_msgs::Odometry base_odom_;
        std::vector<geometry_msgs::PoseStamped> global_plan_;
        std::vector<geometry_msgs::PoseStamped> transformed_plan_;
        std::vector<int> plan_start_end_counter_;
        boost::shared_ptr<EBPlanner> eband_;
        boost::shared_ptr<EBVisualization> eb_visual_;
        boost::shared_ptr<EBTrajectoryCtrl> eb_trj_ctrl_;
        bool goal_reached_;
        bool initialized_;
        boost::mutex odom_mutex_;
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    };
}
#endif //EB_LOCAL_PLANNER_EB_LOCAL_PLANNER_ROS_H
