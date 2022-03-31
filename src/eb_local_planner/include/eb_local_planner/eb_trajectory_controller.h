//
// Created by gk on 2022/2/25.
//

#ifndef EB_LOCAL_PLANNER_EB_TRAJECTORY_CONTROLLER_H
#define EB_LOCAL_PLANNER_EB_TRAJECTORY_CONTROLLER_H
#include <ros/ros.h>
#include <ros/assert.h>
#include <eb_local_planner/conversions_and_types.h>
#include <eb_local_planner/eb_visualization.h>
#include <eb_local_planner/EBPlannerConfig.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <angles/angles.h>
#include <control_toolbox/pid.h>

namespace eb_local_planner{
    class EBTrajectoryCtrl{
    public:
        EBTrajectoryCtrl();
        EBTrajectoryCtrl(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        ~EBTrajectoryCtrl();
        void initialize(std::string name,costmap_2d::Costmap2DROS* costmap_ros);
        void reconfigure(EBPlannerConfig& config);
        void setVisualization(boost::shared_ptr<EBVisualization> target_visual);
        bool setBand(const std::vector<Bubble>& elastic_band);
        bool setOdometry(const nav_msgs::Odometry& odometry);
        bool getTwist(geometry_msgs::Twist& twist_cmd,bool& goal_reached);
        bool getTwistDifferentialDrive(geometry_msgs::Twist twist_cmd,bool& goal_reached);
    private:
        costmap_2d::Costmap2DROS* costmap_ros_;
        boost::shared_ptr<EBVisualization> target_visual_;
        control_toolbox::Pid pid_;
        bool differential_drive_hack_;
        double k_p_,k_nu_,ctrl_freq_;
        double acc_max_,virt_mass_;
        double max_vel_lin_,max_vel_th_,min_vel_lin_,min_vel_th_;
        double min_in_place_vel_th_;
        double in_place_trans_vel_;
        double tolerance_trans_,tolerance_rot_;
        double acc_max_trans_,acc_max_rot_;
        double rotation_correction_threshold_;
        double bubble_velocity_multiplier_;
        double rotation_threshold_multiplier_;
        bool disallow_hysteresis_;
        bool in_final_goal_turn_;
        bool initialized_, band_set_, visualization_;
        std::vector<Bubble> elastic_band_;
        geometry_msgs::Twist odom_vel_;
        geometry_msgs::Twist last_vel_;
        geometry_msgs::Pose ref_frame_band_;

        inline double sign(double n){
            return n < 0.0 ? -1.0:1.0;
        }
        geometry_msgs::Twist getFrame1ToFrame2InRefFrame(const geometry_msgs::Pose& frame1,const geometry_msgs::Pose& frame2,const geometry_msgs::Pose& ref_frame);
        geometry_msgs::Twist getFrame1ToFrame2InRefFrameNew(const geometry_msgs::Pose& frame1,const geometry_msgs::Pose& frame2,const geometry_msgs::Pose& ref_frame);
        geometry_msgs::Twist transformTwistFromFrame1ToFrame2(const geometry_msgs::Twist& current_twist, const geometry_msgs::Pose& frame1,const geometry_msgs::Pose& frame2);
        geometry_msgs::Twist limitTwist(const geometry_msgs::Twist& twist);
        double getBubbleTargetVel(const int& target_bub_num, const std::vector<Bubble>& band,geometry_msgs::Twist& VelDir);
    };
}
#endif //EB_LOCAL_PLANNER_EB_TRAJECTORY_CONTROLLER_H
