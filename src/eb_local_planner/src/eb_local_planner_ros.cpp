//
// Created by gk on 2022/2/25.
//

#include "eb_local_planner/eb_local_planner_ros.h"
#include "pluginlib/class_list_macros.h"
#include "nav_core/base_local_planner.h"

PLUGINLIB_EXPORT_CLASS(eb_local_planner::EBPlannerROS,nav_core::BaseLocalPlanner)

namespace eb_local_planner{
    EBPlannerROS::EBPlannerROS() :costmap_ros_(NULL),tf_(NULL),initialized_(false){}
    EBPlannerROS::EBPlannerROS(std::string name,tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) :
    costmap_ros_(NULL),tf_(NULL),initialized_(false){
        initialize(name,tf,costmap_ros);
    }
    EBPlannerROS::~EBPlannerROS() {}

    void EBPlannerROS::initialize(std::string name,tf2_ros::Buffer* tf,costmap_2d::Costmap2DROS* costmap_ros){
        if (!initialized_){
            costmap_ros_ = costmap_ros;
            tf_ = tf;
            ros::NodeHandle pn("~/" + name);
            g_plan_pub_ = pn.advertise<nav_msgs::Path>("global_plan",1);
            l_plan_pub_ = pn.advertise<nav_msgs::Path>("local_plan",1);
            ros::NodeHandle gn;
            odom_sub_ = gn.subscribe<nav_msgs::Odometry>("odom",1,boost::bind(&EBPlannerROS::odomCallback,this,_1));
            eband_ = boost::shared_ptr<EBPlanner>(new EBPlanner(name,costmap_ros_));
            eb_trj_ctrl_ = boost::shared_ptr<EBTrajectoryCtrl>(new EBTrajectoryCtrl(name,costmap_ros_));
            eb_visual_ = boost::shared_ptr<EBVisualization>(new EBVisualization);
            eband_->setVisualization(eb_visual_);
            eb_trj_ctrl_->setVisualization(eb_visual_);
            eb_visual_->initialize(pn,costmap_ros);

            drs_.reset(new drs(pn));
            drs::CallbackType cb = boost::bind(&EBPlannerROS::reconfigureCallback,this,_1,_2);
            drs_->setCallback(cb);
            initialized_ = true;
            ROS_DEBUG("Elastic Band plugin initialized.");
        }else
            ROS_WARN("This planner has already been initialized, doing nothing.");
    }
    bool EBPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        global_plan_.clear();
        global_plan_ = orig_global_plan;
        std::vector<int> start_end_counts(2,(int)global_plan_.size());
        if (!eb_local_planner::transformGlobalPlan(*tf_,global_plan_,*costmap_ros_,costmap_ros_->getGlobalFrameID(),transformed_plan_,start_end_counts)){
            ROS_WARN("Could not transform the global plan to the frame of the controller");
            return false;
        }
        if (transformed_plan_.empty()){
            ROS_WARN("Transformed plan is empty. Aborting local planner!");
            return false;
        }
        if (!eband_->setPlan(transformed_plan_)){
            costmap_ros_->resetLayers();
            if (!eband_->setPlan(transformed_plan_)){
                ROS_ERROR("Setting plan to Elastic Band method failed!");
                return false;
            }
        }
        ROS_DEBUG("Global plan set to elastic band for optimization");
        plan_start_end_counter_ = start_end_counts;
        eband_->optimizeBand();
        std::vector<eb_local_planner::Bubble> current_band;
        if(eband_->getBand(current_band))
            eb_visual_->publishBand("bubbles",current_band);
        goal_reached_ = false;
        return true;
    }
    bool EBPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        geometry_msgs::PoseStamped global_pose;
        std::vector<geometry_msgs::PoseStamped> tmp_plan;
        ROS_DEBUG("Reading current robot position from costmap and appending it to elastic band.");
        if (!costmap_ros_->getRobotPose(global_pose)){
            ROS_WARN("Could not retrieve up to date robot pose from costmap for local planning.");
            return false;
        }
        tmp_plan.assign(1,global_pose);
        eb_local_planner::AddAtPosition add_frames_at = add_front;
        if (!eband_->addFrames(tmp_plan,add_frames_at)){
            ROS_WARN("Could not connect robot pose to existing elastic band.");
            return false;
        }
        ROS_DEBUG("Checking for new path frames in moving window");
        std::vector<int> plan_start_end_counter = plan_start_end_counter_;
        std::vector<geometry_msgs::PoseStamped> append_transformed_plan;
        if (!eb_local_planner::transformGlobalPlan(*tf_,global_plan_,*costmap_ros_,costmap_ros_->getGlobalFrameID(),transformed_plan_,plan_start_end_counter)){
            ROS_WARN("Could not transform the global plan to the frame of the controller");
            return false;
        }
        if (transformed_plan_.empty()){
            ROS_WARN("Transformed plan is empty. Aborting local planner!");
            return false;
        }
        ROS_DEBUG("Retrieved start-end-counts are: (%d, %d)", plan_start_end_counter.at(0), plan_start_end_counter.at(1));
        ROS_DEBUG("Current start-end-counts are: (%d, %d)", plan_start_end_counter_.at(0), plan_start_end_counter_.at(1));
        append_transformed_plan.clear();
        // cant understand
        if (plan_start_end_counter_.at(1) > plan_start_end_counter.at(1)){
            if (plan_start_end_counter_.at(1) > plan_start_end_counter.at(0)) append_transformed_plan = transformed_plan_;
            else{
                int discarded_frames = plan_start_end_counter.at(0) - plan_start_end_counter_.at(1);
                ROS_ASSERT(transformed_plan_.begin() + discarded_frames + 1 >= transformed_plan_.begin());
                ROS_ASSERT(transformed_plan_.begin() + discarded_frames + 1 < transformed_plan_.end());
                append_transformed_plan.assign(transformed_plan_.begin() + discarded_frames + 1,transformed_plan_.end());
            }
            ROS_DEBUG("Adding %d new frames to current band", (int) append_transformed_plan.size());
            if (eband_->addFrames(append_transformed_plan, add_back)){
                ROS_DEBUG("Successfully added frames to band");
                plan_start_end_counter_ = plan_start_end_counter;
            }else{
                ROS_WARN("Failed to add frames to existing band");
                return false;
            }
        }else
            ROS_DEBUG("Nothing to add");
        ROS_DEBUG("Calling optimization method for elastic band");
        std::vector<eb_local_planner::Bubble> current_band;
        if(!eband_->optimizeBand()){
            ROS_WARN("Optimization failed - Band invalid - No controls available");
            if (!eband_->getBand(current_band))
                eb_visual_->publishBand("bubbles",current_band);
            return false;
        }
        eband_->getBand(current_band);
        if (!eb_trj_ctrl_->setBand(current_band)){
            ROS_DEBUG("Failed to to set current band to Trajectory Controller");
            return false;
        }
        if (!eb_trj_ctrl_->setOdometry(base_odom_)){
            ROS_DEBUG("Failed to to set current odometry to Trajectory Controller");
            return false;
        }
        geometry_msgs::Twist cmd_twist;
        if (!eb_trj_ctrl_->getTwist(cmd_twist,goal_reached_)){
            ROS_DEBUG("Failed to calculate Twist from band in Trajectory Controller");
            return false;
        }
        ROS_DEBUG("Retrieving velocity command: (%f, %f, %f)", cmd_twist.linear.x, cmd_twist.linear.y, cmd_twist.angular.z);
        cmd_vel = cmd_twist;
        std::vector<geometry_msgs::PoseStamped> refined_plan;
        if (eband_->getPlan(refined_plan)){
            // TODO publish local and current gloabl plan
            base_local_planner::publishPlan(refined_plan,g_plan_pub_);
        }
        if (eband_->getBand(current_band))
            eb_visual_->publishBand("bubbles",current_band);
        return true;
    }
    bool EBPlannerROS::isGoalReached(){
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        return goal_reached_;
    }
    void EBPlannerROS::reconfigureCallback(EBPlannerConfig& config,uint32_t level){
        xy_goal_tolerance_ = config.xy_goal_tolerance;
        yaw_goal_tolerance_ = config.yaw_goal_tolerance;
        rot_stopped_vel_ = config.rot_stopped_vel;
        trans_stopped_vel_ = config.trans_stopped_vel;
        if (eband_) eband_->reconfigure(config);
        else ROS_ERROR("Reconfigure CB called before eband planner initialization");
        if (eb_trj_ctrl_) eb_trj_ctrl_->reconfigure(config);
        else ROS_ERROR("Reconfigure CB called before trajectory controller initialization");
        if (eb_visual_) eb_visual_->reconfigure(config);
        else ROS_ERROR("Reconfigure CB called before eband visualizer initialization");
    }
    void EBPlannerROS::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        boost::mutex::scoped_lock lock(odom_mutex_);
        base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
        base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
        base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    }
}