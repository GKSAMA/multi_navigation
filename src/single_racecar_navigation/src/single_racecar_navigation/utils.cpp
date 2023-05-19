/*
 * @Author: Gong, Ke gk.kingj@gmail.com
 * @Date: 2022-11-29 14:10:28
 * @LastEditors: Gong, Ke gk.kingj@gmail.com
 * @LastEditTime: 2022-12-05 11:55:41
 * @FilePath: /src/single_racecar_navigation/src/single_racecar_navigation/utils.cpp
 * @Description: 
 * 
 * Copyright (c) 2022 by Gong, Ke gk.kingj@gmail.com, All Rights Reserved. 
 */
#include "single_racecar_navigation/utils.h"

void SingleNavUtil::ComputeVerticalTarget( std::shared_ptr<ParkingSpace> ps,  std::shared_ptr<CarParam> cp, geometry_msgs::Pose &target, float safe_dist)
{
    float yawt = ps->yaw_ - M_PI_2;
    tf2::Quaternion q;
    q.setRPY(0, 0, yawt);
    tf2::convert(q, target.orientation);
    float xs = 0.0, ys = 0.0, zs = 0.0;
    for(auto p : ps->points_){
        xs += p.x;
        ys += p.y;
        zs += p.z;
    }
    xs /= 4.0;
    ys /= 4.0;
    zs /= 4.0;

    float L1 = cp->min_turn_radius_ + ps->length_ / 2.0;
    float L2 = cp->min_turn_radius_ + cp->wheelbase_ / 2.0 + safe_dist;
    target.position.x = xs - L1 * std::sin(yawt) + L2 * std::cos(yawt);
    target.position.y = ys + L1 * std::cos(yawt) + L2 * std::sin(yawt);
    target.position.z = zs;
}

void SingleNavUtil::ComputeParallelTarget( std::shared_ptr<ParkingSpace> ps,  std::shared_ptr<CarParam> cp, geometry_msgs::Pose &target, float fixed_steer_angle, float fixed_turn_yaw, float dist_safe)
{
    float L1, L2;
    // L1 = ps->width_ / 2.0 + cp->width_ / 2.0 + dist_safe;
    L1 = 2 * cp->wheelbase_ * std::sin(fixed_turn_yaw) + dist_safe;
    float S = (L1 - cp->wheelbase_ * (2 - std::sqrt(2))) * std::sqrt(2);
    L2 = S * std::cos(fixed_turn_yaw) + 2 * cp->wheelbase_ * (1 - std::sin(fixed_turn_yaw)) / std::tan(fixed_steer_angle);
    std::cout << "L1 = " << L1 << "  L2 = " << L2 << std::endl;
    float yawt = ps->yaw_;
    tf2::Quaternion q;
    q.setRPY(0, 0, yawt);
    tf2::convert(q, target.orientation);
    float xs = 0.0, ys = 0.0, zs = 0.0;
    for(auto p : ps->points_){
        xs += p.x;
        ys += p.y;
        zs += p.z;
    }
    xs /= 4.0;
    ys /= 4.0;
    zs /= 4.0;

    target.position.x = xs - L1 * std::sin(yawt) + L2 * std::cos(yawt);
    target.position.y = ys + L1 * std::cos(yawt) + L2 * std::sin(yawt);
    target.position.z = zs;
}