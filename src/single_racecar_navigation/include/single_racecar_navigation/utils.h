/*
 * @Author: Gong, Ke gk.kingj@gmail.com
 * @Date: 2022-11-29 14:10:52
 * @LastEditors: Gong, Ke gk.kingj@gmail.com
 * @LastEditTime: 2022-12-05 11:56:10
 * @FilePath: /src/single_racecar_navigation/include/single_racecar_navigation/utils.h
 * @Description: 
 * 
 * Copyright (c) 2022 by Gong, Ke gk.kingj@gmail.com, All Rights Reserved. 
 */
#ifndef SINGLE_RACECAR_NAVIGATION_UTILS_H_
#define SINGLE_RACECAR_NAVIGATION_UTILS_H_

#include <parking_task/parking.h>

namespace SingleNavUtil{
    void ComputeVerticalTarget(std::shared_ptr<ParkingSpace> ps, std::shared_ptr<CarParam> cp, geometry_msgs::Pose &target, float safe_dist = 0.2);

    void ComputeParallelTarget(std::shared_ptr<ParkingSpace> ps, 
                            std::shared_ptr<CarParam> cp, 
                            geometry_msgs::Pose &target, 
                            float fixed_steer_angle = M_PI_4, 
                            float fixed_turn_yaw = M_PI_4, 
                            float dist_safe = 0.2);
};




#endif