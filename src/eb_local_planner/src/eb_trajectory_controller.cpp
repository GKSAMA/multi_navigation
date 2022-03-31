//
// Created by gk on 2022/2/25.
//

#include "eb_local_planner/eb_trajectory_controller.h"
#include "tf2/utils.h"

namespace eb_local_planner{
    using std::min;
    using std::max;

    EBTrajectoryCtrl::EBTrajectoryCtrl():costmap_ros_(NULL),initialized_(false),band_set_(false),visualization_(false) {}
    EBTrajectoryCtrl::EBTrajectoryCtrl(std::string name, costmap_2d::Costmap2DROS *costmap_ros) :
    costmap_ros_(NULL),initialized_(false),band_set_(false),visualization_(false){
        initialize(name,costmap_ros);
        pid_.initPid(1,0,0,10,-10);
    }
    EBTrajectoryCtrl::~EBTrajectoryCtrl() {}
    void EBTrajectoryCtrl::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros) {
        if (!initialized_){
            ros::NodeHandle node_private("~" + name);
            in_final_goal_turn_ = false;
            costmap_ros_ = costmap_ros;
            last_vel_.linear.x = 0.0;
            last_vel_.linear.y = 0.0;
            last_vel_.linear.z = 0.0;
            last_vel_.angular.x = 0.0;
            last_vel_.angular.y = 0.0;
            last_vel_.angular.z = 0.0;
            geometry_msgs::Pose2D tmp_pose2D;
            tmp_pose2D.x = 0.0;
            tmp_pose2D.y = 0.0;
            tmp_pose2D.theta = 0.0;
            PoseToPose2D(ref_frame_band_,tmp_pose2D);
            initialized_ = true;
        }else{
            ROS_WARN("This planner has already been initialized,doing nothing.");
        }
    }

    void EBTrajectoryCtrl::reconfigure(EBPlannerConfig &config) {
        max_vel_lin_ = config.max_vel_lin;
        max_vel_th_ = config.max_vel_th;
        min_vel_lin_ = config.min_vel_lin;
        min_vel_th_ = config.min_vel_th;
        min_in_place_vel_th_ = config.min_in_place_vel_th;
        in_place_trans_vel_ = config.in_place_trans_vel;
        tolerance_trans_ = config.xy_goal_tolerance;
        tolerance_rot_ = config.yaw_goal_tolerance;
        k_p_ = config.k_prop;
        k_nu_ = config.k_damp;
        ctrl_freq_ = config.Ctrl_Rate;
        acc_max_ = config.max_acceleration;
        virt_mass_ = config.virtual_mass;
        acc_max_trans_ = config.max_translational_acceleration;
        acc_max_rot_ = config.max_rotational_acceleration;
        rotation_correction_threshold_ = config.rotation_correction_threshold;

        differential_drive_hack_ = config.differential_drive;
        bubble_velocity_multiplier_ = config.bubble_velocity_multiplier;
        rotation_threshold_multiplier_ = config.rotation_threshold_multiplier;
        disallow_hysteresis_ = config.disallow_hysteresis;
    }
    void EBTrajectoryCtrl::setVisualization(boost::shared_ptr<EBVisualization> target_visual) {
        target_visual_ = target_visual;
        visualization_ = true;
    }
    bool EBTrajectoryCtrl::setBand(const std::vector<Bubble> &elastic_band) {
        elastic_band_ = elastic_band;
        band_set_ = true;
        return true;
    }
    bool EBTrajectoryCtrl::setOdometry(const nav_msgs::Odometry &odometry) {
        odom_vel_.linear.x = odometry.twist.twist.linear.x;
        odom_vel_.linear.y = odometry.twist.twist.linear.x;
        odom_vel_.linear.z = 0.0;
        odom_vel_.angular.x = 0.0;
        odom_vel_.angular.y = 0.0;
        odom_vel_.angular.z = odometry.twist.twist.angular.z;
        return true;
    }
    double angularDiff(const geometry_msgs::Twist& heading,const geometry_msgs::Pose& pose){
        const double pi = 3.14159265;
        const double t1 = atan2(heading.linear.y,heading.linear.x);
        const double t2 = tf2::getYaw(pose.orientation);
        const double d = t1 - t2;
        if(fabs(d)<pi)
            return d;
        else if (d<0)
            return d+2*pi;
        else
            return d-2*pi;
    }
    bool EBTrajectoryCtrl::getTwistDifferentialDrive(geometry_msgs::Twist twist_cmd, bool &goal_reached) {
        goal_reached = false;
        geometry_msgs::Twist robot_cmd,bubble_diff;
        robot_cmd.linear.x = 0.0;
        robot_cmd.angular.z = 0.0;
        bool command_provided = false;

        if(!initialized_)	{
            ROS_ERROR("Requesting feedforward command from not initialized planner. Please call initialize() before using this planner");
            return false;
        }
        if (!(band_set_)||(elastic_band_.size() < 2)){
            ROS_WARN("Requesting feedforward command from empty band.");
            return false;
        }
        bubble_diff = getFrame1ToFrame2InRefFrameNew(
                elastic_band_.at(0).center.pose,
                elastic_band_.at(1).center.pose,
                elastic_band_.at(0).center.pose);
        float distance_from_goal = -1.0f;
        if (!command_provided){
            int curr_target_bubble = 1;
            while (curr_target_bubble < ((int)elastic_band_.size() - 1)){       // cant understand the meaning of this loop
                curr_target_bubble++;
                bubble_diff = getFrame1ToFrame2InRefFrameNew(elastic_band_.at(0).center.pose,
                                                             elastic_band_.at(curr_target_bubble).center.pose,
                                                             elastic_band_.at(0).center.pose);
            }
            if (!disallow_hysteresis_){
                if (fabs(bubble_diff.linear.x) > tolerance_trans_ ||
                        fabs(bubble_diff.linear.y) > tolerance_trans_){
                    in_final_goal_turn_ = false;
                }
            }
            int goal_bubble = (int)elastic_band_.size() - 1;
            bubble_diff = getFrame1ToFrame2InRefFrameNew(elastic_band_.at(0).center.pose,
                                                         elastic_band_.at(goal_bubble).center.pose,
                                                         elastic_band_.at(0).center.pose);
            distance_from_goal = sqrtf(bubble_diff.linear.x * bubble_diff.linear.x + bubble_diff.linear.y * bubble_diff.linear.y);
            if ((fabs(bubble_diff.linear.x) <= 0.6 * tolerance_trans_ &&
                    fabs(bubble_diff.linear.y) <= 0.6 * tolerance_trans_) ||
                    in_final_goal_turn_){
                double robot_yaw = tf2::getYaw(elastic_band_.at(0).center.pose.orientation);
                double goal_yaw = tf2::getYaw(elastic_band_.at((int)elastic_band_.size() - 1).center.pose.orientation);
                float orientation_diff = angles::normalize_angle(goal_yaw - robot_yaw);
                if (fabs(orientation_diff) > tolerance_rot_){
                    in_final_goal_turn_ = true;
                    ROS_DEBUG("Performing in place rotation for goal (diff): %f", orientation_diff);
                    double rotation_sign = -2 * (orientation_diff < 0) + 1;
                    robot_cmd.angular.z = rotation_sign * min_in_place_vel_th_ + k_p_ * orientation_diff;
                    if (fabs(robot_cmd.angular.z) > max_vel_th_){
                        robot_cmd.angular.z = rotation_sign * max_vel_th_;
                    }
                }else{
                    in_final_goal_turn_ = false;
                    ROS_INFO ("TrajectoryController: Goal reached with distance %.2f, %.2f (od = %.2f); sending zero velocity",
                              bubble_diff.linear.x, bubble_diff.linear.y, orientation_diff);
                    robot_cmd.linear.x = 0.0;
                    robot_cmd.angular.z = 0.0;
                    goal_reached = true;
                }
                command_provided = true;
            }
        }
        bubble_diff = getFrame1ToFrame2InRefFrameNew(elastic_band_.at(0).center.pose,
                                                     elastic_band_.at(1).center.pose,
                                                     elastic_band_.at(0).center.pose);
        if (!command_provided){
            ROS_DEBUG("Goal has not been reached, performing checks to move towards goal");
            double distance_to_next_bubble = sqrt(bubble_diff.linear.x * bubble_diff.linear.x +
                                                    bubble_diff.linear.y * bubble_diff.linear.y);
            double radius_of_next_bubble = 0.7 * elastic_band_.at(1).expansion;
            double in_place_rotation_threshold = rotation_threshold_multiplier_ *
                                                 fabs(atan2(radius_of_next_bubble,distance_to_next_bubble));
            ROS_DEBUG("In-place rotation threshold: %f(%f,%f)",
                      in_place_rotation_threshold, radius_of_next_bubble, distance_to_next_bubble);
            if (fabs(bubble_diff.angular.z) > in_place_rotation_threshold){
                robot_cmd.angular.z = k_p_ * bubble_diff.angular.z;
                double rotation_sign = (bubble_diff.angular.z < 0) ? -1.0: +1.0;
                if (fabs(robot_cmd.angular.z) < min_in_place_vel_th_){
                    robot_cmd.angular.z = rotation_sign * max_vel_th_;
                }
                ROS_DEBUG("Performing in place rotation for start (diff): %f with rot vel: %f", bubble_diff.angular.z, robot_cmd.angular.z);
                command_provided = true;
            }
        }
        if (!command_provided){
            double forward_sign = -2 * (bubble_diff.linear.x < 0) + 1;
            double bubble_radius = 0.7 * elastic_band_.at(0).expansion;
            double velocity_multiplier = bubble_velocity_multiplier_ * bubble_radius;
            double max_vel_lin = max_vel_lin_;
            if (distance_from_goal < 0.75f){
                max_vel_lin = (max_vel_lin < 0.3) ? 0.15 : max_vel_lin / 2;
            }
            double linear_velocity = velocity_multiplier * max_vel_lin;
            linear_velocity *= cos(bubble_diff.angular.z);
            if (fabs(linear_velocity) > max_vel_lin_){
                linear_velocity = forward_sign * max_vel_lin_;
            }else if (fabs(linear_velocity) < min_vel_lin_){
                linear_velocity = forward_sign * min_vel_lin_;
            }
            double error =bubble_diff.angular.z;
            double rotation_sign = -2 * (bubble_diff.angular.z < 0) + 1;
            double angular_velocity = k_p_ * error;
            if (fabs(angular_velocity) > max_vel_th_){
                angular_velocity = rotation_sign * max_vel_th_;
            }else if (fabs(angular_velocity) < min_vel_th_){
                angular_velocity = rotation_sign * min_vel_th_;
            }
            ROS_DEBUG("Selected velocity: lin: %f, ang: %f",
                      linear_velocity, angular_velocity);
            robot_cmd.linear.x = linear_velocity;
            robot_cmd.angular.z = angular_velocity;
            command_provided = true;
        }
        twist_cmd = robot_cmd;
        ROS_DEBUG("Final command: %f, %f", twist_cmd.linear.x, twist_cmd.angular.z);
        return true;
    }
    bool EBTrajectoryCtrl::getTwist(geometry_msgs::Twist &twist_cmd, bool &goal_reached) {
        goal_reached = false;
        if (differential_drive_hack_){
            return getTwistDifferentialDrive(twist_cmd,goal_reached);
        }
        geometry_msgs::Twist robot_cmd, bubble_diff,control_deviation;
        robot_cmd.linear.x = 0.0;
        robot_cmd.linear.y = 0.0;
        robot_cmd.linear.z = 0.0;
        robot_cmd.angular.x = 0.0;
        robot_cmd.angular.y = 0.0;
        robot_cmd.angular.z = 0.0;
        twist_cmd = robot_cmd;
        control_deviation = robot_cmd;
        if (!initialized_){
            ROS_ERROR("Requesting feedforward command from not initialized planner. Please call initialize() before using this planner");
            return false;
        }
        if (!band_set_ || elastic_band_.size() < 2){
            ROS_WARN("Requesting feedforward command from empty band.");
            return false;
        }
        double scaled_radius = 0.7 * elastic_band_.at(0).expansion;
        double bubble_distance,ang_pseudo_dist;
        bubble_diff = getFrame1ToFrame2InRefFrame(elastic_band_.at(0).center.pose,elastic_band_.at(1).center.pose,ref_frame_band_);
        ang_pseudo_dist = bubble_diff.angular.z * getCircumscribedRadius(*costmap_ros_);
        bubble_distance = sqrt(bubble_diff.linear.x * bubble_diff.linear.x + bubble_diff.linear.y * bubble_diff.linear.y + ang_pseudo_dist * ang_pseudo_dist);
        if (visualization_){
            target_visual_->publishBubble("ctrl_target",1,target_visual_->blue,elastic_band_.at(0));
            target_visual_->publishBubble("ctrl_target",2,target_visual_->blue,elastic_band_.at(1));
        }
        double abs_ctrl_dev;
        control_deviation = bubble_diff;
        ang_pseudo_dist = control_deviation.angular.z * getCircumscribedRadius(*costmap_ros_);
        abs_ctrl_dev = sqrt(control_deviation.linear.x * control_deviation.linear.x +
                                    control_deviation.linear.y * control_deviation.linear.y +
                                    ang_pseudo_dist * ang_pseudo_dist);
        if (scaled_radius < bubble_distance){
            double scale_difference = scaled_radius / bubble_distance;
            bubble_diff.linear.x *= scale_difference;
            bubble_diff.linear.y *= scale_difference;
            bubble_diff.angular.z *= scale_difference;
            control_deviation = bubble_diff;
        }
        if (scaled_radius > bubble_distance){
            if (elastic_band_.size() > 2){
                double next_bubble_distance;
                geometry_msgs::Twist next_bubble_diff;
                next_bubble_diff = getFrame1ToFrame2InRefFrame(elastic_band_.at(1).center.pose,
                                                               elastic_band_.at(2).center.pose,
                                                               ref_frame_band_);
                ang_pseudo_dist = next_bubble_diff.angular.z * getCircumscribedRadius(*costmap_ros_);
                next_bubble_distance = sqrt(next_bubble_diff.linear.x * next_bubble_diff.linear.x+
                                                    next_bubble_diff.linear.y * next_bubble_diff.linear.y+
                                                    ang_pseudo_dist * ang_pseudo_dist);
                if (scaled_radius > bubble_distance + next_bubble_distance){
                    control_deviation.linear.x = bubble_diff.linear.x + next_bubble_diff.linear.x;
                    control_deviation.linear.y = bubble_diff.linear.y + next_bubble_diff.linear.y;
                    control_deviation.angular.z = bubble_diff.angular.z + next_bubble_diff.angular.z;
                    if (visualization_)
                        target_visual_->publishBubble("ctrl_target",3,target_visual_->red,elastic_band_.at(2));
                }else{
                    if (visualization_)
                        target_visual_->publishBubble("ctrl_target",3,target_visual_->red,elastic_band_.at(2));
                    double b_distance,cosine_at_bub;
                    double vec_prod,norm_vec1,norm_vec2;
                    double ang_pseudo_dist1,ang_pseudo_dist2;
                    ang_pseudo_dist1 = bubble_diff.angular.z * getCircumscribedRadius(*costmap_ros_);
                    ang_pseudo_dist2 = next_bubble_diff.angular.z * getCircumscribedRadius(*costmap_ros_);
                    vec_prod = -(bubble_diff.linear.x * next_bubble_diff.linear.x +
                            bubble_diff.linear.y * next_bubble_diff.linear.y+
                            ang_pseudo_dist1 * ang_pseudo_dist2);
                    norm_vec1 = sqrt(bubble_diff.linear.x * bubble_diff.linear.x +
                                        bubble_diff.linear.y * bubble_diff.linear.y+
                                        ang_pseudo_dist1 * ang_pseudo_dist1);
                    norm_vec2 = sqrt(next_bubble_diff.linear.x * next_bubble_diff.linear.x +
                                        next_bubble_diff.linear.y * next_bubble_diff.linear.y+
                                        ang_pseudo_dist2 * ang_pseudo_dist2);
                    cosine_at_bub = vec_prod / norm_vec1 / norm_vec2;
                    b_distance = bubble_distance * cosine_at_bub + sqrt(scaled_radius * scaled_radius -
                                                                        bubble_distance * bubble_distance * (1.0-cosine_at_bub*cosine_at_bub));
                    double scaled_next_difference = b_distance / next_bubble_distance;
                    next_bubble_diff.linear.x *= scaled_next_difference;
                    next_bubble_diff.linear.y *= scaled_next_difference;
                    next_bubble_diff.angular.z *= scaled_next_difference;
                    control_deviation.linear.x = bubble_diff.linear.x + next_bubble_diff.linear.x;
                    control_deviation.linear.y = bubble_diff.linear.y + next_bubble_diff.linear.y;
                    control_deviation.angular.z = bubble_diff.angular.z + next_bubble_diff.angular.z;
                }
            }
        }
        ang_pseudo_dist = control_deviation.angular.z * getCircumscribedRadius(*costmap_ros_);
        abs_ctrl_dev = sqrt(control_deviation.linear.x * control_deviation.linear.x +
                control_deviation.linear.y * control_deviation.linear.y +
                ang_pseudo_dist * ang_pseudo_dist);
        if (visualization_){
            geometry_msgs::Pose2D tmp_bubble_2d,curr_bubble_2d;
            geometry_msgs::Pose tmp_pose;
            Bubble new_bubble = elastic_band_.at(0);
            PoseToPose2D(elastic_band_.at(0).center.pose,curr_bubble_2d);
            tmp_bubble_2d.x = curr_bubble_2d.x + control_deviation.linear.x;
            tmp_bubble_2d.y = curr_bubble_2d.y + control_deviation.linear.y;
            tmp_bubble_2d.theta = curr_bubble_2d.theta + control_deviation.angular.z;
            Pose2DToPose(tmp_bubble_2d,tmp_pose);
            new_bubble.center.pose = tmp_pose;
            new_bubble.expansion = 0.1;
            target_visual_->publishBubble("ctrl_target",0,target_visual_->red,new_bubble);
        }
        const geometry_msgs::Point& goal = (--elastic_band_.end())->center.pose.position;
        const double dx = elastic_band_.at(0).center.pose.position.x - goal.x;
        const double dy = elastic_band_.at(0).center.pose.position.y - goal.y;
        const double dist_to_goal = sqrt(dx * dx + dy * dy);
        if(dist_to_goal > rotation_correction_threshold_){
            const double angular_diff = angularDiff(control_deviation,elastic_band_.at(0).center.pose);
            const double vel = pid_.computeCommand(angular_diff,ros::Duration(1/ctrl_freq_));
            const double mult = fabs(vel) > max_vel_th_? max_vel_th_/ fabs(vel):1.0;
            control_deviation.angular.z = vel * mult;
            const double abs_vel = fabs(control_deviation.angular.z);
            ROS_DEBUG_THROTTLE_NAMED (1.0, "angle_correction",
                                      "Angular diff is %.2f and desired angular "
                                      "vel is %.2f.  Initial translation velocity "
                                      "is %.2f, %.2f", angular_diff,
                                      control_deviation.angular.z,
                                      control_deviation.linear.x,
                                      control_deviation.linear.y);
            const double trans_mult = max(0.01,1.0 - abs_vel/max_vel_th_);
            control_deviation.linear.x *= trans_mult;
            control_deviation.linear.y *= trans_mult;
            ROS_DEBUG_THROTTLE_NAMED (1.0, "angle_correction",
                                      "Translation multiplier is %.2f and scaled "
                                      "translational velocity is %.2f, %.2f",
                                      trans_mult, control_deviation.linear.x,
                                      control_deviation.linear.y);
        }else
            ROS_DEBUG_THROTTLE_NAMED (1.0, "angle_correction",
                                      "Not applying angle correction because "
                                      "distance to goal is %.2f", dist_to_goal);
        geometry_msgs::Twist  desired_velocity,currbub_maxvel_dir;
        double desvel_abs,desvel_abs_trans,currbub_maxvel_abs;
        double scale_des_vel;
        desired_velocity = robot_cmd;
        currbub_maxvel_dir = robot_cmd;
        desired_velocity.linear.x = k_p_/k_nu_ * control_deviation.linear.x;
        desired_velocity.linear.y = k_p_/k_nu_ * control_deviation.linear.y;
        desired_velocity.angular.z = k_p_/k_nu_ * control_deviation.angular.z;
        int curr_bub_num = 0;
        currbub_maxvel_abs = getBubbleTargetVel(curr_bub_num,elastic_band_,currbub_maxvel_dir);
        ang_pseudo_dist = desired_velocity.angular.z * getCircumscribedRadius(*costmap_ros_);
        desvel_abs = sqrt(desired_velocity.linear.x * desired_velocity.linear.x + desired_velocity.linear.y * desired_velocity.linear.y +
                ang_pseudo_dist * ang_pseudo_dist);
        if (!desvel_abs > currbub_maxvel_abs){
            scale_des_vel = currbub_maxvel_abs / desvel_abs;
            desired_velocity.linear.x *= scale_des_vel;
            desired_velocity.linear.y *= scale_des_vel;
            desired_velocity.angular.z *= scale_des_vel;
        }
        desvel_abs_trans = sqrt(desired_velocity.linear.x * desired_velocity.linear.x + desired_velocity.linear.y * desired_velocity.linear.y);
        if (desvel_abs_trans > max_vel_lin_){
            scale_des_vel = max_vel_lin_ / desvel_abs_trans;
            desired_velocity.linear.x *= scale_des_vel;
            desired_velocity.linear.y *= scale_des_vel;
            desired_velocity.angular.z *= scale_des_vel;
        }
        if (fabs(desired_velocity.angular.z) > max_vel_th_){
            scale_des_vel = max_vel_th_ / fabs(desired_velocity.angular.z);
            desired_velocity.angular.z *= scale_des_vel;
            desired_velocity.linear.x *= scale_des_vel;
            desired_velocity.linear.y *= scale_des_vel;
        }
        geometry_msgs::Twist acc_desired;
        acc_desired = robot_cmd;
        acc_desired.linear.x = (1.0/virt_mass_) * k_nu_ * (desired_velocity.linear.x - last_vel_.linear.x);
        acc_desired.linear.y = (1.0/virt_mass_) * k_nu_ * (desired_velocity.linear.y - last_vel_.linear.y);
        acc_desired.angular.z = (1.0/virt_mass_) * k_nu_ * (desired_velocity.angular.z - last_vel_.angular.z);
        double scale_acc;
        double abs_acc_trans = sqrt(acc_desired.linear.x * acc_desired.linear.x + acc_desired.linear.y * acc_desired.linear.y);
        if (abs_acc_trans > acc_max_trans_){
            scale_acc = acc_max_trans_ / abs_acc_trans;
            acc_desired.linear.x *= scale_acc;
            acc_desired.linear.y *= scale_acc;
            acc_desired.angular.z *= scale_acc;
        }
        if (fabs(acc_desired.angular.z) > acc_max_rot_){
            scale_acc = fabs(acc_desired.angular.z) / acc_max_rot_;
            acc_desired.angular.z *= scale_acc;
            acc_desired.linear.x *= scale_acc;
            acc_desired.linear.y *= scale_acc;
        }
        last_vel_.linear.x = last_vel_.linear.x + acc_desired.linear.x / ctrl_freq_;
        last_vel_.linear.y = last_vel_.linear.y + acc_desired.linear.y / ctrl_freq_;
        last_vel_.angular.z = last_vel_.angular.z + acc_desired.angular.z / ctrl_freq_;
        last_vel_ = limitTwist(last_vel_);
        robot_cmd = last_vel_;
        robot_cmd = transformTwistFromFrame1ToFrame2(robot_cmd,ref_frame_band_,elastic_band_.at(0).center.pose);
        int curr_target_bubble = 1;
        while(fabs(bubble_diff.linear.x) <= tolerance_trans_ &&
                fabs(bubble_diff.linear.y) <= tolerance_trans_ &&
                fabs(bubble_diff.angular.z) <= tolerance_rot_){
            if (curr_target_bubble < ((int)elastic_band_.size() - 1)){
                curr_target_bubble++;
                bubble_diff = getFrame1ToFrame2InRefFrameNew(elastic_band_.at(0).center.pose,elastic_band_.at(curr_target_bubble).center.pose,ref_frame_band_);
            }else{
                ROS_DEBUG_THROTTLE_NAMED (1.0, "controller_state",
                                          "Goal reached with distance %.2f, %.2f, %.2f"
                                          "; sending zero velocity",
                                          bubble_diff.linear.x, bubble_diff.linear.y,
                                          bubble_diff.angular.z);
                robot_cmd.linear.x = 0.0;
                robot_cmd.linear.y = 0.0;
                robot_cmd.angular.z = 0.0;
                last_vel_.linear.x = 0.0;
                last_vel_.linear.y = 0.0;
                last_vel_.angular.z = 0.0;
                goal_reached = true;
                break;
            }
        }
        twist_cmd = robot_cmd;
        return true;
    }
    double EBTrajectoryCtrl::getBubbleTargetVel(const int &target_bub_num, const std::vector<Bubble> &band,
                                                geometry_msgs::Twist &VelDir) {
        VelDir.linear.x = 0.0;
        VelDir.linear.y = 0.0;
        VelDir.linear.z = 0.0;
        VelDir.angular.x = 0.0;
        VelDir.angular.y = 0.0;
        VelDir.angular.z = 0.0;

        if (target_bub_num >= (int)band.size() - 1) return 0.0;
        double v_max_curr_bub,v_max_next_bub;
        double bubble_distance, angle_to_pseudo_vel,delta_vel_max;
        geometry_msgs::Twist bubble_diff;
        v_max_curr_bub = sqrt(2*elastic_band_.at(target_bub_num).expansion * acc_max_);
        ROS_ASSERT( (target_bub_num >= 0) && ((target_bub_num +1) < (int) band.size()) );
        bubble_diff = getFrame1ToFrame2InRefFrame(band.at(target_bub_num).center.pose,band.at(target_bub_num + 1).center.pose,ref_frame_band_);
        angle_to_pseudo_vel = bubble_diff.angular.z * getCircumscribedRadius(*costmap_ros_);
        bubble_distance = sqrt(bubble_diff.linear.x * bubble_diff.linear.x + bubble_diff.linear.y * bubble_diff.linear.y +
                                angle_to_pseudo_vel * angle_to_pseudo_vel);
        VelDir.linear.x = bubble_diff.linear.x / bubble_distance;
        VelDir.linear.y = bubble_diff.linear.y / bubble_distance;
        VelDir.angular.z = bubble_diff.angular.z / bubble_distance;
        if (bubble_distance > band.at(target_bub_num).expansion) return v_max_curr_bub;
        int next_bub_num = target_bub_num + 1;
        geometry_msgs::Twist dummy_twist;
        v_max_next_bub = getBubbleTargetVel(next_bub_num,band,dummy_twist);
        if (v_max_next_bub >= v_max_curr_bub) return v_max_curr_bub;
        delta_vel_max = sqrt(2* bubble_distance * acc_max_);
        v_max_curr_bub = v_max_next_bub + delta_vel_max;
        return v_max_curr_bub;
    }
    geometry_msgs::Twist EBTrajectoryCtrl::getFrame1ToFrame2InRefFrame(const geometry_msgs::Pose &frame1,
                                                                       const geometry_msgs::Pose &frame2,
                                                                       const geometry_msgs::Pose &ref_frame) {
        geometry_msgs::Pose2D frame1_pose2D,frame2_pose2D,ref_frame_pose2D;
        geometry_msgs::Pose2D frame1_pose2D_rf,frame2_pose2D_rf;
        geometry_msgs::Twist  frame_diff;

        PoseToPose2D(frame1,frame1_pose2D);
        PoseToPose2D(frame2,frame2_pose2D);
        PoseToPose2D(ref_frame,ref_frame_pose2D);
        frame1_pose2D_rf.x = (frame1_pose2D.x - ref_frame_pose2D.x) * cos(ref_frame_pose2D.theta) + (frame1_pose2D.y - ref_frame_pose2D.y) *
                                                                                                         sin(ref_frame_pose2D.theta);
        frame1_pose2D_rf.y = (frame1_pose2D.x - ref_frame_pose2D.x) * sin(ref_frame_pose2D.theta) + (frame1_pose2D.y - ref_frame_pose2D.y) *
                                                                                                         cos(ref_frame_pose2D.theta);
        frame1_pose2D_rf.theta = angles::normalize_angle(frame1_pose2D.theta - frame1_pose2D_rf.theta);
        frame2_pose2D_rf.x = (frame2_pose2D.x - ref_frame_pose2D.x) * cos(ref_frame_pose2D.theta) + (frame2_pose2D.y - ref_frame_pose2D.y) *
                                                                                                         sin(ref_frame_pose2D.theta);
        frame2_pose2D_rf.x = (frame2_pose2D.x - ref_frame_pose2D.x) * cos(ref_frame_pose2D.theta) + (frame2_pose2D.y - ref_frame_pose2D.y) *
                                                                                                         sin(ref_frame_pose2D.theta);
        frame2_pose2D_rf.theta = angles::normalize_angle(frame2_pose2D.theta - frame2_pose2D_rf.theta);
        frame_diff.linear.x = frame2_pose2D_rf.x - frame1_pose2D_rf.x;
        frame_diff.linear.y = frame2_pose2D_rf.y - frame1_pose2D_rf.y;
        frame_diff.linear.z = 0.0;
        frame_diff.angular.x = 0.0;
        frame_diff.angular.x = 0.0;
        frame_diff.angular.z = angles::normalize_angle(frame2_pose2D_rf.theta - frame1_pose2D_rf.theta);
        return frame_diff;
    }
    geometry_msgs::Twist EBTrajectoryCtrl::getFrame1ToFrame2InRefFrameNew(const geometry_msgs::Pose &frame1,
                                                                          const geometry_msgs::Pose &frame2,
                                                                          const geometry_msgs::Pose &ref_frame) {
        double x1 = frame1.position.x - ref_frame.position.x;
        double y1 = frame1.position.y - ref_frame.position.y;
        double x2 = frame2.position.x - ref_frame.position.x;
        double y2 = frame2.position.y - ref_frame.position.y;
        double yaw_ref = tf2::getYaw(ref_frame.orientation);
        double x_diff = x2 - x1;
        double y_diff = y2 - y1;
        double theta_diff = atan2(y_diff,x_diff);
        double rotation = angles::normalize_angle(yaw_ref);
        double x_final = x_diff * cos(rotation) + y_diff * sin(rotation);
        double y_final = -x_diff * sin(rotation) + y_diff * cos(rotation);

        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = x_final;
        twist_msg.linear.y = y_final;
        twist_msg.angular.z = angles::normalize_angle(theta_diff - yaw_ref);
        return twist_msg;
    }
    geometry_msgs::Twist EBTrajectoryCtrl::limitTwist(const geometry_msgs::Twist &twist) {
        geometry_msgs::Twist res = twist;
        double lin_overshoot = sqrt(res.linear.x * res.linear.x + res.linear.y * res.linear.y) / max_vel_lin_;
        double lin_undershoot = min_vel_lin_ / sqrt(res.linear.x * res.linear.x + res.linear.y * res.linear.y);
        if (lin_overshoot > 1.0){
            res.linear.x /= lin_overshoot;
            res.linear.y /= lin_overshoot;
            res.angular.z /= lin_overshoot;
        }
        if (lin_undershoot > 1.0){
            res.linear.x *= lin_undershoot;
            res.linear.y *= lin_undershoot;
        }
        if (fabs(res.angular.z) > max_vel_th_){
            double scale = max_vel_th_ / fabs(res.angular.z);
            res.angular.z *= scale;
            res.linear.x *= scale;
            res.linear.y *= scale;
        }
        if (fabs(res.angular.z) < min_vel_th_) res.angular.z = min_vel_th_ * sign(res.angular.z);
        if (sqrt(twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y) < in_place_trans_vel_){
            if (fabs(res.angular.z) < min_in_place_vel_th_)
                res.angular.z = min_in_place_vel_th_ * sign(res.angular.z);
            res.linear.x = 0.0;
            res.linear.y = 0.0;
        }
        ROS_DEBUG("Angular command %f", res.angular.z);
        return res;
    }
}