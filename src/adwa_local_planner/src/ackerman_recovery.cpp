#include "adwa_local_planner/ackerman_recovery.h"
#include <pluginlib/class_list_macros.h>
#include <nav_core/parameter_magic.h>
#include <tf2/utils.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <string>

PLUGINLIB_EXPORT_CLASS(ackerman_recovery::AckermanRecovery, nav_core::RecoveryBehavior)

namespace ackerman_recovery{
    AckermanRecovery::AckermanRecovery() : local_costmap_(NULL),initialized_(false),world_model_(NULL)
    {
    }

    AckermanRecovery::initialize(std::string name, tf2_ros::Buffer*, 
                                 costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap)
    {
        if(!initialized_){
            local_costmap_ = local_costmap;

            ros::NodeHandle private_nh("~/" + name);
            ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

            private_nh.param("sim_granularity",sim_granularity_, 0.017);
            private_nh.param("frequency", frequency_, 20.0);
            max_trans_vel_ = nav_core::loadParameterWithDeprecation(blp_nh,"max_trans_vel", "max_trans_velocity", 2.0);
            min_trans_vel_ = nav_core::loadParameterWithDeprecation(blp_nh,"min_trans_vel", "min_trans_velocity", 2.0);

            world_model_ = new base_local_planner::CosmapModel(*local_costmap_->getCostmap());
            initialized_ = true;
        } else {
            ROS_ERROR("You should not call initialize twice on this object, doing nothing");
        }
    }

    AckermanRecovery::~AckermanRecovery()
    {
        delete world_model_;
    }

    void AckermanRecovery::runBehavior()
    {
        if(!initialized_){
            ROS_ERROR("This object must be initialized before runBehavior is called");
            return ;
        } 
        if(local_costmap_ == NULL){
            ROS_ERROR("The costmap passed to the RotateRecovery object cannot be NULL. Doing nothing.");
            return;
        }
        ROS_WARN("Rotate recovery behavior started.");
        ros::Rate r(frequency_);
        ros::Nodehandle n;
        ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        geometry_msgs::PoseStamped global_pose;
        local_costmap_->getRobotPose(global_pose);
        double current_angle = tf::getYaw(global_pose.pose.orientation);
        double start_angle = current_angle;
    }
};