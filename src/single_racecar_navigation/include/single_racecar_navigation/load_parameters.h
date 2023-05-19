/*
 * @Author: Gong, Ke gk.kingj@gmail.com
 * @Date: 2022-11-29 14:56:04
 * @LastEditors: Gong, Ke gk.kingj@gmail.com
 * @LastEditTime: 2023-01-10 15:42:24
 * @FilePath: /src/single_racecar_navigation/include/single_racecar_navigation/load_parameters.h
 * @Description: load car and parking space parameters from yaml file
 * 
 * Copyright (c) 2022 by Gong, Ke gk.kingj@gmail.com, All Rights Reserved. 
 */
#ifndef SINGLE_RACECAR_NAVIGATION_LOAD_PARAMETERS_H_
#define SINGLE_RACECAR_NAVIGATION_LOAD_PARAMETERS_H_

#include <string>
#include <memory>
#include <parking_task/parking.h>

#include "single_racecar_navigation/ParkingSpaceInfo.h"

namespace SingleNavUtils{
    class Configurations {
    public:
        // Configurations(){}
        Configurations(std::string pathToNode);
        ~Configurations() = default;

        void loadParameters(std::string pathToNode);

        void loadTargetInfo(single_racecar_navigation::ParkingSpaceInfo *parking_space_info);
        
        void targetCB(const single_racecar_navigation::ParkingSpaceInfo::ConstPtr& msg);
        
        std::shared_ptr<ParkingSpace> GetParkingSpace()
        {
            return parkingSpace;
        }

        std::shared_ptr<CarParam> GetCarParam()
        {
            return carParam;
        }

        float GetSafeDist()
        {
            return dist_safe;
        }

        float GetParkingSpaceType()
        {
            return parking_space_type;
        }

        bool ReceiveTarget()
        {
            return receiveTarget;
        }

    private:
        std::shared_ptr<ParkingSpace> parkingSpace;
        std::shared_ptr<CarParam> carParam;
        float dist_safe;
        int parking_space_type;
        single_racecar_navigation::ParkingSpaceInfo *parking_space_info;
        bool receiveTarget;
    };
};


#endif //SINGLE_RACECAR_NAVIGATION_LOAD_PARAMETERS_H_