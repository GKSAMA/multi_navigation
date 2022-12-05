#include "adwa_local_planner/adwa_trajectory_sample_generator.h"

#include <cmath>
#include <iostream>
#include <algorithm> 
#include <ros/console.h>

#include <base_local_planner/velocity_iterator.h>

AdwaTrajectoryGenerator::AdwaTrajectoryGenerator()
{
    limits_ = nullptr;
}

void AdwaTrajectoryGenerator::initialise(const Eigen::Vector3f& pos, 
                                        const Eigen::Vector3f& state, 
                                        const Eigen::Vector3f& goal,
                                        AdwaLimits* limits, 
                                        const Eigen::Vector2f& vsamples,
                                        bool discretize_bt_time)
{
    double steer_angle,steer_vel,trans_vel;
    double max_steer_vel,min_steer_vel;
    double max_steer_angle,min_steer_angle;
    double max_trans_vel,min_trans_vel;
    double T = 0.0;
    double threshold = 1e-3;
    pos_ = pos;
    state_ = state;
    limits_ = limits;
    // std::cout << "limit in trajectory generator:" << std::endl;
    // std::cout << *limits_;
    next_sample_index_ = 0;
    sample_params_.clear();
    trans_vel = state[0];
    steer_angle = state[1];
    steer_vel = state[2];
    // Distance between current position and goal position.
    double dist=sqrt((goal[0] - pos[0]) * (goal[0] - pos[0]) + (goal[1] - pos[1]) * (goal[1] - pos[1]));
    // if go straight to goal to cost minimum time.
    sim_time_ = dist / limits_->max_trans_vel + limits_->max_trans_vel / limits_->max_trans_acc;

    sim_time_ = sim_time_ > max_sim_time_ ? max_sim_time_ : sim_time_;
    sim_time_ = sim_time_ < min_sim_time_ ? min_sim_time_ : sim_time_;
    
    //set angular velocity limits
    max_steer_angle = limits->max_steer_vel * sim_time_ + steer_angle;
    min_steer_angle = limits->min_steer_vel * sim_time_ + steer_angle;
    max_steer_angle = std::min(max_steer_angle, limits_->max_steer_angle);
    min_steer_angle = std::max(min_steer_angle, limits_->min_steer_angle);

    // set translation velocity limits
    // the time that accelerate to max velocity from current velocity
    double TAccToMax = (limits->max_trans_vel - trans_vel) / limits->max_trans_acc;
    // the time that deacccelerate to zero velocity from max velocity
    double TDeaccMaxToZero = limits->max_trans_vel / limits->max_trans_acc; 
    if((sim_time_ - TAccToMax - TDeaccMaxToZero) > 0.0){
        max_trans_vel = limits->max_trans_vel;
    } else {
        max_trans_vel = (sim_time_ * limits->max_trans_acc + trans_vel) / 2.0;
    }
    double TDeaccToMin = -(limits->min_trans_vel - trans_vel) / limits->max_trans_acc;
    double TAccMinToZero = -limits->min_trans_vel / limits->max_trans_acc;
    if((sim_time_ - TAccMinToZero - TDeaccToMin) > 0.0){
        min_trans_vel = limits->min_trans_vel;
    } else {
        min_trans_vel = -(sim_time_ * limits->max_trans_acc - trans_vel) / 2.0;
    }
    Eigen::Vector2f sample = Eigen::Vector2f::Zero();
    base_local_planner::VelocityIterator trans_vel_it(min_trans_vel, max_trans_vel, vsamples[0]);
    base_local_planner::VelocityIterator steer_angle_it(min_steer_angle, max_steer_angle, vsamples[1]);
    // ROS_INFO("window shape: maxTransV = %.2f, minTransV = %.2f, maxSteerAngle = %.3f, minSteerAngle = %.3f", max_trans_vel, min_trans_vel, max_steer_angle, min_steer_angle);
    for(; !trans_vel_it.isFinished(); trans_vel_it++){
        sample[0] = trans_vel_it.getVelocity();
        for(; !steer_angle_it.isFinished(); steer_angle_it++){
            sample[1] = steer_angle_it.getVelocity();
            sample_params_.emplace_back(sample);
        }
        steer_angle_it.reset();
    }
    // ROS_INFO("has %d trajectories.", sample_params_.size());
}

void AdwaTrajectoryGenerator::setParameters(double max_sim_time, 
                                            double min_sim_time,
                                            double sim_granularity,
                                            double angular_sim_granularity,
                                            double sim_period)
{
    max_sim_time_ = max_sim_time;
    min_sim_time_ = min_sim_time;
    sim_granularity_ = sim_granularity;
    angular_sim_granularity_ = angular_sim_granularity;
    sim_period_ = sim_period;
}

bool AdwaTrajectoryGenerator::hasMoreTrajectories()
{
    return next_sample_index_ < sample_params_.size();
}

bool AdwaTrajectoryGenerator::nextTrajectory(base_local_planner::Trajectory& trajectory)
{
    bool result = false;
    if(hasMoreTrajectories()){
        if(generateTrajectory(pos_,state_,sample_params_[next_sample_index_],trajectory)){
            result = true;	
        }
    }
    ++next_sample_index_;
    return result;
}

bool AdwaTrajectoryGenerator::generateTrajectory(Eigen::Vector3f pos, 
                                                 Eigen::Vector3f state, 
                                                 Eigen::Vector2f sample_target_vel,
                                                 base_local_planner::Trajectory& traj)
{
    double eps = 1e-3;
    double x_i = pos[0];
    double y_i = pos[1];
    double theta_i = pos[2];

    double trans_vel_i = state[0];
    double steer_angle_i = state[1];
    double steer_vel_i = state[2];
    traj.cost_ = -1.0;
    traj.resetPoints();
    int num_steps = ceil(sim_time_ / sim_granularity_);
    if(num_steps == 0){
        num_steps = 1;
    }
    double speed = 0.0, angle = 0.0;
    if((limits_->min_trans_vel >= 0 && sample_target_vel[0] + eps < limits_->min_trans_vel) && 
        (limits_->min_steer_angle >= 0 && fabs(sample_target_vel[1] + eps < limits_->min_steer_angle))){
        return false;
    }

    if(limits_->max_trans_vel >= 0 && sample_target_vel[0] - eps > limits_->max_trans_vel){
        return false;
    }

    // compute a timestep
    double dt = sim_time_ / num_steps;
    double time = 0.0;
    traj.time_delta_ = dt;
    traj.xv_ = sample_target_vel[0];
    traj.yv_ = 0;
    traj.thetav_ = sample_target_vel[1];
    Eigen::Vector3f position = {x_i, y_i, theta_i};
    Eigen::Vector2f loop_vel = {trans_vel_i, steer_angle_i};
    for(int i = 0; i < num_steps; ++i){
        traj.addPoint(position[0], position[1], position[2]);
        loop_vel = computeNewVelocity(loop_vel[0], loop_vel[1], dt, sample_target_vel, steer_vel_i);
        position = computeNewPosition(position, loop_vel, dt);
    }
    return true;
}

Eigen::Vector2f AdwaTrajectoryGenerator::computeNewVelocity(double trans_vel, 
                                   double steer_angle, 
                                   double dt, 
                                   Eigen::Vector2f sample_target_vel,
                                   double &steer_vel)
{
    Eigen::Vector2f new_vel = Eigen::Vector2f::Zero();
    double eps = 1e-3;
    if(trans_vel < sample_target_vel[0]){
        new_vel[0] = std::min(double(sample_target_vel[0]), trans_vel + limits_->max_trans_acc * dt);
    } else {
        new_vel[0] = std::max(double(sample_target_vel[0]), trans_vel - limits_->max_trans_acc * dt);
    }
    if(steer_angle + eps < sample_target_vel[1]){
        if(steer_vel + dt * limits_->max_steer_acc <= limits_->max_steer_vel){
            new_vel[1] = steer_angle + steer_vel * dt + limits_->max_steer_acc * dt * dt / 2.0;
            steer_vel += dt * limits_->max_steer_acc;
        }else if(steer_vel <= limits_->max_steer_vel){
            double leftTime = (limits_->max_steer_vel - steer_vel) / limits_->max_steer_acc;
            new_vel[1] = steer_angle + steer_vel * leftTime + limits_->max_steer_acc * leftTime * leftTime / 2.0;
            steer_vel += leftTime * limits_->max_steer_acc;
        }
    } else if(steer_angle - eps > sample_target_vel[1]){
        if(steer_vel - dt * limits_->max_steer_acc >= limits_->min_steer_vel) {
            new_vel[1] = steer_angle - steer_vel * dt - limits_->max_steer_acc * dt * dt / 2.0;
            steer_vel -= dt * limits_->max_steer_acc;
        } else if(steer_vel >= limits_->min_steer_vel) {
            double leftTime = (limits_->max_steer_vel - steer_vel) / limits_->max_steer_acc;
            new_vel[1] = steer_angle + steer_vel * leftTime + limits_->max_steer_acc * leftTime * leftTime / 2.0;
            steer_vel -= leftTime * limits_->max_steer_acc;
        }
    } else{
        new_vel[1] = steer_angle;
        steer_vel = 0.0;
    }
    return new_vel;
}

Eigen::Vector3f AdwaTrajectoryGenerator::computeNewPosition(const Eigen::Vector3f& pos, 
                                   const Eigen::Vector2f& vel,
                                   double dt)
{
    Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
    double r, d;
    if(vel[1] > 0.001){
        r = fabs(limits_->wheelbase / tan(vel[1]));
        d = vel[0] * dt;
        if(vel[1] > 0){
            new_pos[2] = pos[2] + d / r;
        } else {
            new_pos[2] = pos[2] - d / r;
        }
        new_pos[0] = pos[0] + d * cos(vel[1]);
        new_pos[1] = pos[1] + d * sin(vel[1]);
    } else {
        d = vel[0] * dt;
        new_pos[0] = pos[0] + d * cos(vel[1]);
        new_pos[1] = pos[1] + d * sin(vel[1]);
    }
    return new_pos;
}

AdwaTrajectoryGenerator::~AdwaTrajectoryGenerator()
{}