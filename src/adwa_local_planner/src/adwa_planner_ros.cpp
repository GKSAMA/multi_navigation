#include <adwa_planner_ros.h>
#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>

#include <nav_core/parameter_magic.h>

PLUGINLIB_EXPORT_CLASS(adwa_local_planner::ADWAPlannerROS,nav_core::BaseLocalPlanner)

namespace adwa_local_planner{
    void ADWAPlannerROS::reconfigureCB(dwa_local_planner::DWAPlannerConfig &config, uint32_t level) {
        if (setup_ && config.restore_defaults){
            config = default_config_;
            config.restore_defaults = false;
        }
        if (!setup_){
            default_config_ = config;
            setup_ = true;
        }

        // update generic local planner params
        base_local_planner::LocalPlannerLimits limits;
        limits.max_vel_trans = config.max_vel_trans;
        limits.min_vel_trans = config.min_vel_trans;
        limits.max_vel_x = config.max_vel_x;
        limits.min_vel_x = config.min_vel_x;
        limits.max_vel_y = config.max_vel_y;
        limits.min_vel_y = config.min_vel_y;
        limits.max_vel_theta = config.max_vel_theta;
        limits.min_vel_theta = config.min_vel_theta;
        limits.acc_lim_x = config.acc_lim_x;
        limits.acc_lim_y = config.acc_lim_y;
        limits.acc_lim_theta = config.acc_lim_theta;
        limits.acc_lim_trans = config.acc_lim_trans;
        limits.xy_goal_tolerance = config.xy_goal_tolerance;
        limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
        limits.prune_plan = config.prune_plan;
        limits.trans_stopped_vel = config.trans_stopped_vel;
        limits.theta_stopped_vel = config.theta_stopped_vel;
        planner_util_.reconfigureCB(limits, config.restore_defaults);

        dp_->reconfigure(config);
        max_acc_theta = limits.acc_lim_theta;
    }

    ADWAPlannerROS::ADWAPlannerROS() :
    initialized_(false),
    odom_helper_("odom"),
    setup_(false){}

    void ADWAPlannerROS::initialize(std::string name,
                                    tf2_ros::Buffer *tf,
                                    costmap_2d::Costmap2DROS *costmap_ros) {
        if (!isInitialized()){
            ros::NodeHandle private_nh("~" + name);
            g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
            l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
            tf_ = tf;
            costmap_ros_ = costmap_ros;
            costmap_ros_->getRobotPose(current_pose_);

            // update costmap
            costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
            planner_util_.initialize(tf, costmap,costmap_ros_->getGlobalFrameID());
            // create the actual planner
            dp_ = boost::shared_ptr<ADWAPlanner>(new ADWAPlanner(name, &planner_util_));

            if (private_nh.getParam("odom_topic", odom_topic_))
                odom_helper_.setOdomTopic(odom_topic_);

            initialized_ = true;

            nav_core::warnRenamedParameter(private_nh, "max_vel_trans", "max_trans_vel");
            nav_core::warnRenamedParameter(private_nh, "min_vel_trans", "min_trans_vel");
            nav_core::warnRenamedParameter(private_nh, "max_vel_theta", "max_rot_vel");
            nav_core::warnRenamedParameter(private_nh, "min_vel_theta", "min_rot_vel");
            nav_core::warnRenamedParameter(private_nh, "acc_lim_trans", "acc_limit_trans");
            nav_core::warnRenamedParameter(private_nh, "theta_stopped_vel", "rot_stopped_vel");

            dsrv_ = new dynamic_reconfigure::Server<dwa_local_planner::DWAPlannerConfig>(private_nh);
            dynamic_reconfigure::Server<dwa_local_planner::DWAPlannerConfig>::CallbackType cb = boost::bind(&ADWAPlannerROS::reconfigureCB, this, _1, _2);
            dsrv_->setCallback(cb);
        }else{
            ROS_WARN("This planner has already been initialized, doing nothing.");
        }
    }

    bool ADWAPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan) {
        if (!isInitialized()){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        latchedStopRotateController_.resetLatching();
        // ROS_INFO("Got new plan");
        return dp_->setPlan(orig_global_plan);
    }

    bool ADWAPlannerROS::isGoalReached() {
        if (!isInitialized()){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        if(!costmap_ros_->getRobotPose(current_pose_)){
            ROS_ERROR("Could not get robot pose");
            return false;
        }

        if(latchedStopRotateController_.isGoalReached(&planner_util_,odom_helper_,current_pose_)){
            ROS_INFO("Goal reached");
            return true;
        }else {
            return false;
        }
    }

    void ADWAPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped> &path) {
        base_local_planner::publishPlan(path,l_plan_pub_);
    }

    void ADWAPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped> &path) {
        base_local_planner::publishPlan(path,g_plan_pub_);
    }

    ADWAPlannerROS::~ADWAPlannerROS() {
        delete dsrv_;
    }

    bool ADWAPlannerROS::adwaComputeVelocityCommands(geometry_msgs::PoseStamped &global_pose,
                                                     geometry_msgs::Twist &cmd_vel) {
        if(!isInitialized()){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        geometry_msgs::PoseStamped robot_vel;
        odom_helper_.getRobotVel(robot_vel);

        // compute what trajectory tp drive along
        geometry_msgs::PoseStamped drive_cmds;
        drive_cmds.header.frame_id = costmap_ros_->getBaseFrameID();
        // transfer acc_theta data
        drive_cmds.pose.position.x = max_acc_theta;

        // call with update footprint
        base_local_planner::Trajectory path = dp_->findBestPath(global_pose, robot_vel, drive_cmds);

        //pass along drive commands
        cmd_vel.linear.x = drive_cmds.pose.position.x;
        cmd_vel.linear.y = drive_cmds.pose.position.y;
        cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);

        // if robot cannot move
        std::vector<geometry_msgs::PoseStamped> local_plan;
        if (path.cost_ < 0){
            ROS_DEBUG_NAMED("dwa_local_planner",
                            "The dwa local planner failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot.");
            local_plan.clear();
            // publish an empty path
            publishLocalPlan(local_plan);
            return false;
        }
        ROS_DEBUG_NAMED("dwa_local_planner", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.",
                        cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
        // Fill out the local plan
        for(unsigned int i = 0; i < path.getPointsSize(); ++i){
            double p_x, p_y, p_th;
            path.getPoint(i, p_x, p_y, p_th);

            geometry_msgs::PoseStamped p;
            p.header.frame_id = costmap_ros_->getGlobalFrameID();
            p.header.stamp = ros::Time::now();
            p.pose.position.x = p_x;
            p.pose.position.y = p_y;
            p.pose.position.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, p_th);
            tf2::convert(q, p.pose.orientation);
            local_plan.push_back(p);
        }

        publishLocalPlan(local_plan);
        return true;
    }

    bool ADWAPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
        // dispatches to either dwa sampling control or stop and rotate control, depending on whether we have been close enough to goal
        if (!costmap_ros_->getRobotPose(current_pose_)){
            ROS_ERROR("Could not get robot pose");
            return false;
        }
        std::vector<geometry_msgs::PoseStamped> transformed_plan;
        if (!planner_util_.getLocalPlan(current_pose_,transformed_plan)){
            ROS_ERROR("Could not get local plan");
            return false;
        }
        if (transformed_plan.empty()){
            ROS_WARN_NAMED("dwa_local_planner", "Received an empty transformed plan.");
            return false;
        }
        ROS_DEBUG_NAMED("dwa_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());

        dp_->updatePlanAndLocalCosts(current_pose_,
                                     transformed_plan,
                                     costmap_ros_->getRobotFootprint());
        if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_)){
            // publish an empty plan because we've reached our goal position
            std::vector<geometry_msgs::PoseStamped> local_plan;
            std::vector<geometry_msgs::PoseStamped> transformed_plan;
            publishGlobalPlan(transformed_plan);
            publishLocalPlan(local_plan);
            base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
            return latchedStopRotateController_.computeVelocityCommandsStopRotate(
                    cmd_vel,
                    limits.getAccLimits(),
                    dp_->getSimPeriod(),
                    &planner_util_,
                    odom_helper_,
                    current_pose_,
                    boost::bind(&ADWAPlanner::checkTrajectory, dp_, _1, _2, _3));
        }else{
            bool isOk = adwaComputeVelocityCommands(current_pose_, cmd_vel);
            ROS_INFO("Commands: x: %.2f, y: %.2f, theta: %.2f", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
            if (isOk){
                publishGlobalPlan(transformed_plan);
            }else{
                ROS_WARN_NAMED("dwa_local_planner", "DWA planner failed to produce path.");
                std::vector<geometry_msgs::PoseStamped> empty_plan;
                publishGlobalPlan(empty_plan);
            }
            return isOk;
        }
    }
}