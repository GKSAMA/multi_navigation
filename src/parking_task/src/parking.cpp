/*
 * @Author: Gong, Ke gk.kingj@gmail.com
 * @Date: 2022-11-22 13:39:49
 * @LastEditors: Gong, Ke gk.kingj@gmail.com
 * @LastEditTime: 2023-04-12 21:28:25
 * @FilePath: /src/parking_task/src/parking.cpp
 * @Description: 
 * 
 * Copyright (c) 2022 by Gong, Ke gk.kingj@gmail.com, All Rights Reserved. 
 */
#include "parking_task/parking.h"

#include <visualization_msgs/Marker.h>
#include <angles/angles.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <boost/bind.hpp>

void ParkingTask::fpCB(const nav_msgs::Odometry::ConstPtr& odom)
{
    this->carState->pose.position.x = odom->pose.pose.position.x;
    this->carState->pose.position.y = odom->pose.pose.position.y;
    this->carState->pose.position.z = odom->pose.pose.position.z;
    this->carState->pose.orientation.x = odom->pose.pose.orientation.x;
    this->carState->pose.orientation.y = odom->pose.pose.orientation.y;
    this->carState->pose.orientation.z = odom->pose.pose.orientation.z;
    this->carState->pose.orientation.w = odom->pose.pose.orientation.w;
    // std::cout << "get odom! car state: " << " x=" << carState->pose.position.x << " y=" << carState->pose.position.y << std::endl;
}

void ParkingTask::rearCB(const geometry_msgs::PoseStamped::ConstPtr& odom)
{
    this->rearState->pose.position.x = odom->pose.position.x;
    this->rearState->pose.position.y = odom->pose.position.y;
    this->rearState->pose.position.z = odom->pose.position.z;
    this->rearState->pose.orientation.x = odom->pose.orientation.x;
    this->rearState->pose.orientation.y = odom->pose.orientation.y;
    this->rearState->pose.orientation.z = odom->pose.orientation.z;
    this->rearState->pose.orientation.w = odom->pose.orientation.w;
    // std::cout << "get rear! rear state: " << " x=" << rearState->pose.position.x << " y=" << rearState->pose.position.y << std::endl;
}

void ParkingTask::cmdCB(const geometry_msgs::Twist::ConstPtr& cmd)
{
    cur_cmd_vel.linear.x = cmd->linear.x;
    cur_cmd_vel.angular.z = cmd->angular.z;
}


void ParkingTask::printCmd(const ros::TimerEvent &event){
    std::cout << "cmd vel x=" << cmd_vel.linear.x << " steer=" << cmd_vel.angular.z << std::endl;
    // std::cout << "cur vel x=" << cur_cmd_vel.linear.x << " steer=" << cur_cmd_vel.angular.z << std::endl;
}

ParkingTask::ParkingTask(ros::NodeHandle &n, std::string robotName)
{
    odom_sub = n.subscribe<nav_msgs::Odometry>("/" + robotName + "/odom", 10, boost::bind(&ParkingTask::fpCB, this, _1));
    rear_sub = n.subscribe<geometry_msgs::PoseStamped>("/" + robotName + "/rear_pose", 10, boost::bind(&ParkingTask::rearCB, this, _1));
    cmd_pub = n.advertise<geometry_msgs::Twist>("/" + robotName + "/cmd_vel", 1);
    cmd_sub = n.subscribe<geometry_msgs::Twist>("/" + robotName + "/cmd_vel", 10, boost::bind(&ParkingTask::cmdCB, this, _1));
    parking_space_pub = n.advertise<visualization_msgs::Marker>("/parking_space", 1);
    timer = n.createTimer(ros::Duration(0.5), boost::bind(&ParkingTask::printCmd, this, _1));
    carState = new geometry_msgs::PoseStamped();
    rearState = new geometry_msgs::PoseStamped();
    // ros::spinOnce();
}

ParkingTask::~ParkingTask()
{
    
}

void ParkingTask::SetParkingSpace(const std::shared_ptr<ParkingSpace> &ps){
    this->parkingSpace = ps;
    this->stopPoint.position.x = (ps->points_[0].x + ps->points_[1].x + ps->points_[2].x + ps->points_[3].x) / 4.0;
    this->stopPoint.position.y = (ps->points_[0].y + ps->points_[1].y + ps->points_[2].y + ps->points_[3].y) / 4.0;
    this->stopPoint.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0,0,ps->yaw_);
    tf2::convert(q, this->stopPoint.orientation);
}

void ParkingTask::SetSafeDistance(double dist_safe)
{
    if(dist_safe > 0){
        safe_dist = dist_safe;
    } else {
        safe_dist = (this->parkingSpace->length_ - this->carParam->length_) / 2.0;
        safe_dist = std::max(safe_dist, (this->parkingSpace->width_ - this->carParam->width_) / 2.0);
    }
}

void ParkingTask::ShowParkingSpace()
{
    visualization_msgs::Marker p;
    p.header.frame_id = "/map";
    p.header.stamp = ros::Time::now();
    p.ns = "parkingSpace";
    p.action = visualization_msgs::Marker::ADD;
    p.type = visualization_msgs::Marker::CUBE;
    tf2::Quaternion q;
    q.setRPY(0,0,this->parkingSpace->yaw_);
    tf2::convert(q, p.pose.orientation);
    p.pose.position.x = this->stopPoint.position.x;
    p.pose.position.y = this->stopPoint.position.y;
    p.pose.position.z = this->stopPoint.position.z;
    p.scale.x = this->parkingSpace->length_;
    p.scale.y = this->parkingSpace->width_;
    p.scale.z = 0.1;
    p.color.a = 1.0;
    p.color.r = 1.0;
    p.color.g = 0.0;
    p.color.b = 0.0;
    parking_space_pub.publish(p);
}

bool ParkingTask::VerticalTask(float vx)
{
    ros::spinOnce();
    float k = tan(this->parkingSpace->yaw_);
    float b = this->stopPoint.position.y - k * this->stopPoint.position.x;
    float dist = std::sqrt(std::pow(this->carState->pose.position.x - this->stopPoint.position.x, 2) 
                            + std::pow(this->carState->pose.position.y - this->stopPoint.position.y, 2));
    float distToParking_p;
    // distToParking_p = 0.8;
    if(std::fabs(this->parkingSpace->yaw_ - M_PI_2) < 1e-3 || std::fabs(this->parkingSpace->yaw_ + M_PI_2) < 1e-3){
        distToParking_p = std::fabs(this->carState->pose.position.x - this->stopPoint.position.x) 
                            - this->carParam->axis_to_center_;
    } else {
        distToParking_p = std::fabs(k * this->carState->pose.position.x - this->carState->pose.position.y + b) / std::sqrt(1 + k * k) 
                            - this->carParam->axis_to_center_ 
                            + this->safe_dist;

    }
    float distToParking_v = std::sqrt(std::pow(dist, 2) - std::pow(distToParking_p, 2));

    float pos1 = this->carState->pose.position.x - this->carParam->axis_to_center_;
    float distance = 0;
    float x = this->carState->pose.position.x;
    float y = this->carState->pose.position.y;

    ros::Timer();
    std::cout << "wheelbase=" << this->carParam->wheelbase_ 
              << " distToParking_p=" << distToParking_p 
              << " distToParking_v=" << distToParking_v 
              << " min_turn_radius_=" << this->carParam->min_turn_radius_ <<std::endl;
    if(distToParking_p >= this->carParam->min_turn_radius_ && distToParking_v >= this->carParam->min_turn_radius_){
        this->ShowParkingSpace();
        this->parkingSpace->print();
        float steer_angle = std::fabs(atan(this->carParam->wheelbase_ / distToParking_p));
        // float steer_angle = 0.785;
        std::cout << "***********************************************************************" << std::endl;
        std::cout << "stop point: x=" << this->stopPoint.position.x << " y=" << this->stopPoint.position.y << std::endl;
        std::cout << "car position: x=" << this->carState->pose.position.x << " y=" << this->carState->pose.position.y << std::endl;
        std::cout << "wheelbase=" << this->carParam->wheelbase_ << " distToParking_p=" << distToParking_p << " steer_angle=" << steer_angle <<std::endl;
        std::cout << "***********************************************************************" << std::endl;
        float cur_yaw = tf2::getYaw(this->carState->pose.orientation);
        // float yaw_diff = angles::normalize_angle_positive(cur_yaw) - 
        //                  angles::normalize_angle_positive(this->parkingSpace->yaw_);
        float yaw_diff = cur_yaw - this->parkingSpace->yaw_;
        std::cout << "cur yaw = " << cur_yaw << " parking yaw = " << this->parkingSpace->yaw_ << " yaw diff = " << yaw_diff << std::endl;
        if(yaw_diff < 0) {
            steer_angle = -steer_angle;
        }
        float angle_velocity = vx / distToParking_p;
        float time1 = yaw_diff / angle_velocity;
        // curve segment
        std::cout << "turn task started!" << std::endl;
        while(ros::ok()){
            // std::cout << "car position: " << " x=" << carState->pose.position.x << " y=" << carState->pose.position.y << std::endl;
            // std::cout << "car state: " << " x=" << cur_cmd_vel.linear.x << " th=" << cur_cmd_vel.angular.z << std::endl;
            ros::spinOnce();
            cur_yaw = tf2::getYaw(this->carState->pose.orientation);
            yaw_diff = cur_yaw - this->parkingSpace->yaw_;
            // std::cout << "cur yaw=" << cur_yaw << " target yaw="<< this->parkingSpace->yaw_ << " yaw_diff: " << yaw_diff << std::endl;
            // car is same with the parking space direction, end this segment
            if(std::fabs(yaw_diff) < 0.01){
                while(cur_cmd_vel.angular.z != 0){
                // while(ros::ok()){
                    ros::spinOnce();
                    std::cout << "turn task finished!" << std::endl;
                    std::cout << "raidus = " << pos1 - this->carState->pose.position.x << std::endl;
                    std::cout << "distance = " << distance << std::endl;
                    std::cout << "radius = " << (distance / M_PI) / 2.0 << std::endl;
                    CmdVelPub(0,0);
                    ros::Duration(0.01).sleep();
                }
                break;
            }
            distance += std::sqrt(std::pow(this->carState->pose.position.x - x, 2) + std::pow(this->carState->pose.position.y - y, 2));
            CmdVelPub(vx, steer_angle);

            // fast frequency cause topic value update with delay
            ros::Duration(0.01).sleep();
        }
        std::cout << "turn task finished!" << std::endl;
        this->cmd_pub.publish(this->cmd_vel);
        std::cout << "straight task started!" << std::endl;
        // straight segment
        while(ros::ok()){
            ros::spinOnce();
            float dist_diff = pow(this->carState->pose.position.x - this->stopPoint.position.x, 2) + 
                                pow(this->carState->pose.position.y - this->stopPoint.position.y, 2);
            // car position close to the stop point, stop the car
            if(dist_diff < 1e-2){
                while(cur_cmd_vel.linear.x != 0){
                    ros::spinOnce();
                    std::cout << "straight task finished!" << std::endl;
                    CmdVelPub(0,0);
                    ros::Duration(0.01).sleep();
                }
                break;
            }
            CmdVelPub(vx,0);
            ros::Duration(0.01).sleep();
        }

        // drive into vertical parking space task finished. 
        this->Stop();
        return true;
    } else {
        ROS_ERROR("Can't drive in parking space with one try!");
        return false;
    }
}

bool ParkingTask::ParallelTask(float vx)
{
    ros::spinOnce();
    std::cout << "task started ! " << std::endl;
    float k = -1 / tan(this->parkingSpace->yaw_);
    geometry_msgs::Pose rear_stop_point;
    float temp1 = this->carParam->wheelbase_ * std::cos(this->parkingSpace->yaw_) / 2;
    float temp2 = this->carParam->wheelbase_ * std::sin(this->parkingSpace->yaw_)/ 2;
    rear_stop_point.position.x = this->stopPoint.position.x - temp1;
    rear_stop_point.position.y = this->stopPoint.position.y - temp2;
    std::cout << "rear_stop_point finished ! x = " << rear_stop_point.position.x << " y = " << rear_stop_point.position.y << std::endl;

    float b = rear_stop_point.position.y - k * rear_stop_point.position.x;
    // std::cout << "b = " <<b <<std::endl;
    std::cout << "get rear! rear state: " << " x=" << rearState->pose.position.x << " y=" << rearState->pose.position.y << std::endl;
    float dist = std::sqrt(std::pow(this->rearState->pose.position.x - rear_stop_point.position.x, 2) + std::pow(this->rearState->pose.position.y - rear_stop_point.position.y, 2));
    std::cout << "dist = " <<dist <<std::endl;
    float distToParking_v, distToParking_p;
    // judge parking space is vertical (yaw approach 1.57)
    bool isParkingParallel = std::fabs(this->parkingSpace->yaw_) < 0.01 || std::fabs(this->parkingSpace->yaw_ - M_PI) < 0.01 || std::fabs(this->parkingSpace->yaw_ + M_PI) < 0.01;
    bool isParkingVertical = std::fabs(this->parkingSpace->yaw_ - M_PI_2) < 0.01 || std::fabs(this->parkingSpace->yaw_ + M_PI_2) < 0.01;
    if(isParkingParallel){
        distToParking_v = std::fabs(this->rearState->pose.position.x - rear_stop_point.position.x);
        distToParking_p = std::fabs(this->rearState->pose.position.y - rear_stop_point.position.y);
    } else {
        distToParking_v = std::fabs(k * this->rearState->pose.position.x - this->rearState->pose.position.y + b) / std::sqrt(1 + k * k);
        distToParking_p = std::sqrt(std::pow(dist, 2) - std::pow(distToParking_v, 2));
    }
    std::cout << "distToParking_v = " <<distToParking_v <<std::endl;
    // float distToParking_p = 0.8;
    std::cout << "distToParking_p = " <<distToParking_p <<std::endl;
    float steering_angle = 0.785;

    // for racecar rear_wheel_to_back is the diameter of the wheel
    float rear_wheel_to_back = 0.073;
    float rear_wheel_to_head = this->carParam->length_ - rear_wheel_to_back;
    // h is the distance between the car and the wall
    float h = 1.0;
    float phi = M_PI / 4;
    float R = this->carParam->min_turn_radius_;
    float S = (distToParking_v - 2 * R * std::sin(phi)) / std::cos(phi);
    std::cout << "S = " <<S <<std::endl;
    std::cout << "S * std::sin(phi) + 2 * R * (1 - std::cos(phi)) = " << S * std::sin(phi) + 2 * R * (1 - std::cos(phi)) << std::endl;
    // if(S > 0 && std::fabs(distToParking_p - S * std::sin(phi) + 2 * R * (1 - std::cos(phi))) <= safe_dist){
    if(S > 0){
        std::cout << "distance is legal ! " << std::endl;
        this->ShowParkingSpace();
        this->parkingSpace->print();
        std::cout << "stop point: x=" << rear_stop_point.position.x << " y=" << rear_stop_point.position.y << std::endl;
        std::cout << "car rear position: x=" << this->rearState->pose.position.x << " y=" << this->rearState->pose.position.y << std::endl;
        std::cout << "wheelbase=" << this->carParam->wheelbase_ << " distToParking_v=" << distToParking_v << " steer_angle=" << steering_angle <<std::endl;
        float cur_yaw = tf2::getYaw(this->rearState->pose.orientation);
        std::cout << "parkingSpace->yaw = " <<this->parkingSpace->yaw_ <<std::endl;
        // kl is the slope of the line in parking space yaw
        float kl = tan(this->parkingSpace->yaw_);
        std::cout << "kl = " <<kl <<std::endl;
        float b1 = rear_stop_point.position.y - kl * rear_stop_point.position.x;
        std::cout << "b1 = " <<b1 <<std::endl;
        float b3 = this->rearState->pose.position.y - kl * this->rearState->pose.position.x;
        std::cout << "b3 = " <<b3 <<std::endl;
        float deltaD = R * (1 - std::cos(phi)) * std::sqrt(1 + kl * kl);
        float segment_yaw;
        float yaw_diff;
        // this flag is use in the case that parking yaw approach 1.57, 
        // when in this case the line is a vertical line which ignore the slope.
        bool carInLeft = false;
        if(kl >= 0){
            if(b1 > b3){
                segment_yaw = cur_yaw - phi;
                steering_angle = steering_angle;
                deltaD = -deltaD;
            } else {
                segment_yaw = cur_yaw + phi;
                steering_angle = -steering_angle;
                carInLeft = true;
            }
        }else{
            if(b1 > b3){
                segment_yaw = cur_yaw + phi;
                steering_angle = -steering_angle;
                deltaD = -deltaD;
            } else {
                segment_yaw = cur_yaw - phi;
                steering_angle = steering_angle;
                carInLeft = true;
            }
        }
        float b2 = b1 + deltaD;
        std::cout << "b2 = " <<b2 <<std::endl;
        std::cout << "steer command = " << steering_angle <<std::endl;
        // curve1 segment
        while(ros::ok()){
            ros::spinOnce();
            cur_yaw = tf2::getYaw(this->rearState->pose.orientation);
            yaw_diff = std::fabs(cur_yaw - segment_yaw);
            std::cout << "cur_yaw = " <<cur_yaw << " segment_yaw = " << segment_yaw <<  " yaw_diff = " << yaw_diff <<std::endl;
            if(yaw_diff < 0.01){
                CmdVelPub(0,0);
                while(cur_cmd_vel.angular.z != 0){
                    ros::spinOnce();
                    CmdVelPub(0,0);
                    ros::Duration(0.01).sleep();
                }
                break;
            }
            CmdVelPub(vx, steering_angle);
            ros::Duration(0.01).sleep();
        }
        std::cout << "curve 1 segment finished!" << std::endl;

        while (ros::ok())
        {
            ros::spinOnce();
            float temp;
            if(isParkingVertical){
                temp = carInLeft? R * (std::cos(phi) - 1) : R * (1 - std::cos(phi));
                temp += rear_stop_point.position.x;
                std::cout << "rear x = " <<this->rearState->pose.position.x << " temp = " << temp <<std::endl;
            } else {
                temp = kl * this->rearState->pose.position.x + b2;
                std::cout << "rear y = " <<this->rearState->pose.position.y << " temp = " << temp <<std::endl;
            }
            if(std::fabs(this->rearState->pose.position.y - temp) < 4e-3 // normal line
                || std::fabs(this->rearState->pose.position.x - temp) < 4e-3){ // vertical line
                CmdVelPub(0,0);
                while(cur_cmd_vel.angular.x != 0){
                    ros::spinOnce();
                    CmdVelPub(0,0);
                    ros::Duration(0.01).sleep();
                }
                break;
            }
            CmdVelPub(vx, 0);
            ros::Duration(0.01).sleep();
        }
        std::cout << "straight segment finished!" << std::endl;

        while(ros::ok()){
            ros::spinOnce();
            cur_yaw = tf2::getYaw(this->rearState->pose.orientation);
            yaw_diff = cur_yaw - this->parkingSpace->yaw_;
            if(std::fabs(yaw_diff) < 0.01){
                CmdVelPub(0,0);
                while(cur_cmd_vel.angular.z != 0){
                    ros::spinOnce();
                    CmdVelPub(0,0);
                    ros::Duration(0.01).sleep();
                }
                break;
            }
            CmdVelPub(vx, -steering_angle);
            ros::Duration(0.01).sleep();
        }
        std::cout << "curve 2 segment finished!" << std::endl;

        while(ros::ok()){
            ros::spinOnce();
            float dist_diff = pow(this->carState->pose.position.x - this->stopPoint.position.x, 2) + 
                                pow(this->carState->pose.position.y - this->stopPoint.position.y, 2);
            if(dist_diff < 1e-2){
                CmdVelPub(0,0);
                while(cur_cmd_vel.angular.x != 0){
                    ros::spinOnce();
                    CmdVelPub(0,0);
                    ros::Duration(0.01).sleep();
                }
                break;
            }
            CmdVelPub(-vx, 0);
            ros::Duration(0.01).sleep();
        }
        std::cout << "adjust segment finished!" << std::endl;
        std::cout << "task finished!" << std::endl;
        return true;
    } else {
        ROS_ERROR("Can't drive in parking space with one try!");
        return false;
    }
}

bool ParkingTask::run(float vx)
{
    
}

bool ParkingTask::Stop()
{
    CmdVelPub(0,0);
    return true;
}

void ParkingTask::test(float vx, float steer_angle)
{
    float dist = 0;
    float x = this->carState->pose.position.x;
    float y = this->carState->pose.position.y;
    float th = tf2::getYaw(this->carState->pose.orientation);
    std::cout << "x = " << x << " y = " << y << " th = " << th << std::endl; 
    while (ros::ok())
    {
        ros::spinOnce();
        th = tf2::getYaw(this->carState->pose.orientation);
        // std::cout << "th = " << th;
        // std::cout << " thDiff = " << th - M_PI << std::endl;
        if(std::fabs(th - M_PI) < 0.01){
        // if(std::fabs(th) < 0.01){
            this->cmd_vel.linear.x = 0;
            this->cmd_vel.angular.z = 0;
            this->cmd_pub.publish(this->cmd_vel);
            std::cout << "dist = " << dist << std::endl;
            float thmax = std::atan(1.868 / (dist / M_PI));
            std::cout << "radius = " << dist / M_PI << std::endl;
            std::cout << "max steer angle = " << thmax*180/M_PI << std::endl;
            break;
        }
        dist += std::sqrt(std::pow(this->carState->pose.position.x - x, 2) + std::pow(this->carState->pose.position.y - y, 2));
        x = this->carState->pose.position.x;
        y = this->carState->pose.position.y;
        CmdVelPub(vx, steer_angle);
        ros::Duration(0.01).sleep();
    }
}

void ParkingTask::CmdVelPub(float vx, float steer_angle)
{
    this->cmd_vel.linear.x = vx;
    this->cmd_vel.linear.y = this->cmd_vel.linear.z = 0;
    this->cmd_vel.angular.x = this->cmd_vel.angular.y = 0;
    this->cmd_vel.angular.z = steer_angle;
    cmd_pub.publish(this->cmd_vel);
}
