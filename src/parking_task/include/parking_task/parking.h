/*
 * @Author: Gong, Ke gk.kingj@gmail.com
 * @Date: 2022-11-21 23:20:05
 * @LastEditors: Gong, Ke gk.kingj@gmail.com
 * @LastEditTime: 2022-12-03 18:52:53
 * @FilePath: /src/parking_task/include/parking_task/parking.h
 * @Description: 
 * 
 * Copyright (c) 2022 by Gong, Ke gk.kingj@gmail.com, All Rights Reserved. 
 */
#ifndef PARKING_H_
#define PARKING_H_

#include <iostream>
#include <vector>
#include <memory>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>

struct ParkingSpace
{
    float length_;
    float width_;
    float yaw_;
    std::vector<geometry_msgs::Point> points_ = std::vector<geometry_msgs::Point>(4,geometry_msgs::Point());
    ParkingSpace(){}
    /**
     * @description: 
     * @param len
     * @param w
     * @param th
     */
    ParkingSpace(float len, float w, float th):length_(len),width_(w),yaw_(th){}
    /**
     * @description: 
     * @param len
     * @param w
     * @param th
     * @param points
     */    
    ParkingSpace(float len, float w, float th, std::vector<geometry_msgs::Point> points){
        this->length_ = len;
        this->width_ = w;
        this->yaw_ = th;
        this->points_ = points;
    }
    /**
     * @description: 
     * @param ps
     */    
    ParkingSpace(const ParkingSpace &ps){
        this->length_ = ps.length_;
        this->width_ = ps.width_;
        this->yaw_ = ps.yaw_;
        this->points_ = ps.points_;
    }

    void SetParams(float len, float w, float th, std::vector<geometry_msgs::Point> points){
        length_ = len;
        width_ = w;
        yaw_ = th;
        points_ = points;
    }

    void print(){
        std::cout << "points: ";
        for(auto p : points_){
            std::cout << " x=" << p.x << " y=" << p.y;
        }
        std::cout << std::endl;
    }
};

struct CarParam
{
    float length_;
    float width_;
    float wheelbase_;
    float axis_length_;
    float max_steer_angle_;
    float min_turn_radius_;
    float axis_to_center_;

    CarParam(){}
    /**
     * @description: 
     * @param {float} len
     * @param {float} w
     * @param {float} wheelbase
     * @param {float} axisL
     * @param {float} max_steer_angle
     * @return {*}
     */
    CarParam(float len, float w, float wheelbase, float axisL, float max_steer_angle)
        :length_(len), width_(w), wheelbase_(wheelbase), axis_length_(axisL), max_steer_angle_(max_steer_angle)
    {
        this->min_turn_radius_ = this->wheelbase_ / std::tan(this->max_steer_angle_);
        this->axis_to_center_ = this->wheelbase_ / 2;
    }
    /**
     * @description: 
     * @param {CarParam&} cp
     * @return {*}
     */
    CarParam(const CarParam& cp)
    {
        this->length_ = cp.length_;
        this->width_ = cp.width_;
        this->wheelbase_ = cp.wheelbase_;
        this->axis_length_ = cp.axis_length_;
        this->max_steer_angle_ = cp.max_steer_angle_;
        this->min_turn_radius_ = cp.min_turn_radius_;
        this->axis_to_center_ = this->wheelbase_ / 2;
    }
    
    /**
     * @description: 
     * @param {float} len
     * @param {float} w
     * @param {float} wheelbase
     * @param {float} axisL
     * @param {float} max_steer_angle
     * @return {*}
     */
    void SetParams(float len, float w, float wheelbase, float axisL, float max_steer_angle)
    {
        this->length_ = len;
        this->width_ = w;
        this->wheelbase_ = wheelbase;
        this->axis_length_ = axisL;
        this->max_steer_angle_ = max_steer_angle;
        this->min_turn_radius_ = this->wheelbase_ / std::tan(this->max_steer_angle_);
        this->axis_to_center_ = this->wheelbase_ / 2;
    }

    void SetAxisToCenter(float distance)
    {
        this->axis_to_center_ = distance;
    }
};

class ParkingTask{
public:
    /**
     * @description: 
     */
    ParkingTask();
    
    /**
     * @description: 
     * @param n
     */
    ParkingTask(ros::NodeHandle &n, std::string robotName);

    /**
     * @description: 
     */
    ~ParkingTask();

    void fpCB(const nav_msgs::Odometry::ConstPtr& odom);

    void cmdCB(const geometry_msgs::Twist::ConstPtr& cmd);
    
    void rearCB(const geometry_msgs::PoseStamped::ConstPtr& odom);

    void printCmd(const ros::TimerEvent &event);

    /**
     * @description: 
     * @param curState
     */
    void SetCarState(const geometry_msgs::PoseStamped *curState){
        this->carState->pose.position.x = curState->pose.position.x;
        this->carState->pose.position.y = curState->pose.position.y;
        this->carState->pose.position.z = curState->pose.position.z;
        this->carState->pose.orientation.x = curState->pose.orientation.x;
        this->carState->pose.orientation.y = curState->pose.orientation.y;
        this->carState->pose.orientation.z = curState->pose.orientation.z;
        this->carState->pose.orientation.w = curState->pose.orientation.w;
        float cur_yaw = tf2::getYaw(this->carState->pose.orientation);
        this->rearState->pose.position.x = carState->pose.position.x - std::cos(cur_yaw) * 0.167;
        this->rearState->pose.position.y = carState->pose.position.y - std::sin(cur_yaw) * 0.167;
        this->rearState->pose.position.z = carState->pose.position.z;
        this->rearState->pose.orientation = carState->pose.orientation;
    }

    /**
     * @description: 
     * @param ps
     */
    void SetParkingSpace(const std::shared_ptr<ParkingSpace> &ps);

    /**
     * @description: 
     * @param {CarParam} &cp
     * @return {*}
     */
    void SetCarParam(const std::shared_ptr<CarParam> &cp){
        this->carParam = cp;
    }

    /**
     * @description: 
     * @param {float} dist_safe
     */
    void SetSafeDistance(double dist_safe);

    /**
     * @description: 
     * @return {*}
     */
    void ShowParkingSpace();

    /**
     * @description: 
     * @param {float} vx
     * @return {*}
     */
    bool VerticalTask(float vx);

    /**
     * @description: 
     * @param {float} vx
     * @return {*}
     */
    bool ParallelTask(float vx);

    /**
     * @description: 
     * @param {float} vx
     * @return {*}
     */
    bool run(float vx);

    bool Stop();

    void test(float vx, float steer_angle);

    void CmdVelPub(float vx, float steer_angle);
private:
    ros::Subscriber odom_sub;
    ros::Subscriber rear_sub;
    ros::Subscriber cmd_sub;
    ros::Publisher cmd_pub;
    ros::Publisher parking_space_pub;
    ros::Timer timer;
    double safe_dist;
    std::shared_ptr<ParkingSpace> parkingSpace;
    std::shared_ptr<CarParam> carParam;
    geometry_msgs::PoseStamped *carState;
    geometry_msgs::PoseStamped *rearState;
    geometry_msgs::Pose stopPoint;
    geometry_msgs::Twist cmd_vel;
    geometry_msgs::Twist cur_cmd_vel;
};

#endif // PARKING_H_