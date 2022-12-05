#ifndef ADWA_LIMITS_H
#define ADWA_LIMITS_H

#include <Eigen/Core>
#include <iostream>

class AdwaLimits
{
public:
    //dwa
    double max_vel_x;
    double min_vel_x;
    double max_vel_y;
    double min_vel_y;
    double max_vel_theta;
    double min_vel_theta;
    double acc_lim_x;
    double acc_lim_y;
    double acc_lim_theta;
    double acc_lim_trans;
    double theta_stopped_vel;
    // translational limits
    double max_trans_vel;
    double min_trans_vel;
    double max_trans_acc;
    // steering limits
    double max_steer_angle;
    double min_steer_angle;
    double max_steer_vel;
    double min_steer_vel;
    double max_steer_acc;
    // kinematic attributes
    double wheelbase;
    double wheel_distance;
    double wheel_radius; 
    // general planner limits
    bool prune_plan;
    double xy_goal_tolerance;
    double yaw_goal_tolerance;
    double trans_stopped_vel;
    double rot_stopped_vel;

    AdwaLimits(){}
    AdwaLimits(
        // dwa
        double max_vel_x,
        double min_vel_x,
        double max_vel_y,
        double min_vel_y,
        double max_vel_theta,
        double min_vel_theta,
        double acc_lim_x,
        double acc_lim_y,
        double acc_lim_theta,
        double acc_lim_trans,
        double theta_stopped_vel,
        // translational limits
        double max_trans_vel,
        double min_trans_vel,
        double max_trans_acc,
        // steering limits
        double max_steer_angle,
        double min_steer_angle,
        double max_steer_vel,
        double min_steer_vel,
        double max_steer_acc,
        // kinematic attributes
        double wheelbase,
        double wheel_distance,
        double wheel_radius,
        // general planner limits
        double xy_goal_tolerance,
        double yaw_goal_tolerance,
        bool prune_plan = true,
        double trans_stopped_vel = 0.1,
        double rot_stopped_vel = 0.1):
        max_vel_x(max_vel_x),
        min_vel_x(min_vel_x),
        max_vel_y(max_vel_y),
        min_vel_y(min_vel_y),
        max_vel_theta(max_vel_theta),
        min_vel_theta(min_vel_theta),
        acc_lim_x(acc_lim_x),
        acc_lim_y(acc_lim_y),
        acc_lim_theta(acc_lim_theta),
        acc_lim_trans(acc_lim_trans),
        theta_stopped_vel(theta_stopped_vel),
        max_trans_vel(max_trans_vel),
        min_trans_vel(min_trans_vel),
        max_trans_acc(max_trans_acc),
        max_steer_angle(max_steer_angle),
        min_steer_angle(min_steer_angle),
        max_steer_vel(max_steer_vel),
        min_steer_vel(min_steer_vel),
        max_steer_acc(max_steer_acc),
        wheelbase(wheelbase),
        wheel_distance(wheel_distance),
        wheel_radius(wheel_radius),
        prune_plan(prune_plan),
        xy_goal_tolerance(xy_goal_tolerance),
        yaw_goal_tolerance(yaw_goal_tolerance),
        trans_stopped_vel(trans_stopped_vel),
        rot_stopped_vel(rot_stopped_vel){}

    ~AdwaLimits(){}

    friend std::ostream & operator<<(std::ostream &cout, AdwaLimits &limits){
        std::cout << "max_vel_x :" << limits.max_vel_x << std::endl;
        std::cout << "min_vel_x :" << limits.min_vel_x << std::endl;
        std::cout << "max_vel_y :" << limits.max_vel_y << std::endl;
        std::cout << "min_vel_y :" << limits.min_vel_y << std::endl;
        std::cout << "max_vel_theta :" << limits.max_vel_theta << std::endl;
        std::cout << "min_vel_theta :" << limits.min_vel_theta << std::endl;
        std::cout << "acc_lim_x :" << limits.acc_lim_x << std::endl;
        std::cout << "acc_lim_y :" << limits.acc_lim_y << std::endl;
        std::cout << "acc_lim_theta :" << limits.acc_lim_theta << std::endl;
        std::cout << "acc_lim_trans :" << limits.acc_lim_trans << std::endl;
        std::cout << "theta_stopped_vel :" << limits.theta_stopped_vel << std::endl;
        std::cout << "max_trans_vel :" << limits.max_trans_vel << std::endl;
        std::cout << "min_trans_vel :" << limits.min_trans_vel << std::endl;
        std::cout << "max_trans_acc :" << limits.max_trans_acc << std::endl;
        std::cout << "max_steer_angle :" << limits.max_steer_angle << std::endl;
        std::cout << "min_steer_angle :" << limits.min_steer_angle << std::endl;
        std::cout << "max_steer_vel :" << limits.max_steer_vel << std::endl;
        std::cout << "min_steer_vel :" << limits.min_steer_vel << std::endl;
        std::cout << "max_steer_acc :" << limits.max_steer_acc << std::endl;
        std::cout << "wheelbase :" << limits.wheelbase << std::endl;
        std::cout << "wheel_distance :" << limits.wheel_distance << std::endl;
        std::cout << "wheel_radius :" << limits.wheel_radius << std::endl;
        std::cout << "prune_plan :" << limits.prune_plan << std::endl;
        std::cout << "xy_goal_tolerance :" << limits.xy_goal_tolerance << std::endl;
        std::cout << "yaw_goal_tolerance :" << limits.yaw_goal_tolerance << std::endl;
        std::cout << "trans_stopped_vel :" << limits.trans_stopped_vel << std::endl;
        std::cout << "rot_stopped_vel :" << limits.rot_stopped_vel << std::endl;
    }
};

#endif // ADWA_LIMITS_H