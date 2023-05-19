/*
 * @Author: Gong, Ke gk.kingj@gmail.com
 * @Date: 2022-11-29 14:55:49
 * @LastEditors: Gong, Ke gk.kingj@gmail.com
 * @LastEditTime: 2023-01-10 15:42:36
 * @FilePath: /src/single_racecar_navigation/src/single_racecar_navigation/load_parameters.cpp
 * @Description: 
 * 
 * Copyright (c) 2022 by Gong, Ke gk.kingj@gmail.com, All Rights Reserved. 
 */
#include <yaml-cpp/yaml.h>
#include "single_racecar_navigation/load_parameters.h"

namespace SingleNavUtils{
    Configurations::Configurations(std::string pathToPackage)
    {
        std::cout << "create configurations" << std::endl;
        parkingSpace = std::make_shared<ParkingSpace>();
        carParam = std::make_shared<CarParam>();
        receiveTarget = false;
        loadParameters(pathToPackage);
    }

    void Configurations::loadParameters(std::string pathToPackage)
    {
        YAML::Node config = YAML::LoadFile(pathToPackage + "/param/parameters_config.yaml");
        parking_space_type = config["parking_space_type"].as<int>();
        float length = config["parking_space"]["len"].as<float>();
        float width = config["parking_space"]["width"].as<float>();
        float yaw = config["parking_space"]["yaw"].as<float>();
        std::cout << "parking space: len = " << length << " width = " << width << " yaw = " << yaw << std::endl;
        std::vector<std::vector<float>> points = config["parking_space"]["points"].as<std::vector<std::vector<float>>>();
        std::vector<geometry_msgs::Point> p = std::vector<geometry_msgs::Point>(4, geometry_msgs::Point());
        for(int i = 0; i < 4; ++i){
            p[i].x = points[i][0];
            p[i].y = points[i][1];
            p[i].z = points[i][2];
        }
        parkingSpace->SetParams(length, width, yaw, p);

        length = config["car_parameters"]["len"].as<float>();
        width = config["car_parameters"]["width"].as<float>();
        float axis_length = config["car_parameters"]["axis_len"].as<float>();
        float wheelbase = config["car_parameters"]["wheelbase"].as<float>();
        float max_steer_angle = config["car_parameters"]["max_turn_angle"].as<float>();
        dist_safe = config["safe_dist"].as<float>();
        carParam->SetParams(length, width, wheelbase, axis_length, max_steer_angle);
    }

    void Configurations::loadTargetInfo(single_racecar_navigation::ParkingSpaceInfo *parking_space_info)
    {
        parking_space_type = parking_space_info->parking_space_type;
        std::vector<geometry_msgs::Point> p = std::vector<geometry_msgs::Point>(4, geometry_msgs::Point());
        p[0].x = parking_space_info->p1.x;
        p[0].y = parking_space_info->p1.y;
        p[0].z = parking_space_info->p1.z;
        p[1].x = parking_space_info->p2.x;
        p[1].y = parking_space_info->p2.y;
        p[1].z = parking_space_info->p2.z;
        p[2].x = parking_space_info->p3.x;
        p[2].y = parking_space_info->p3.y;
        p[2].z = parking_space_info->p3.z;
        p[3].x = parking_space_info->p4.x;
        p[3].y = parking_space_info->p4.y;
        p[3].z = parking_space_info->p4.z;
        parkingSpace->SetParams(parking_space_info->length,
                                parking_space_info->width,
                                parking_space_info->yaw,
                                p);
    }

    void Configurations::targetCB(const single_racecar_navigation::ParkingSpaceInfo::ConstPtr& msg)
    {
        // parking_space_info->p1 = msg->p1;
        // parking_space_info->p2 = msg->p2;
        // parking_space_info->p3 = msg->p3;
        // parking_space_info->p4 = msg->p4;
        // parking_space_info->yaw = msg->yaw;
        // parking_space_info->parking_space_type = msg->parking_space_type;
        // parking_space_info->length = msg->length;
        // parking_space_info->width = msg->width;

        parking_space_type = msg->parking_space_type;
        std::vector<geometry_msgs::Point> p = std::vector<geometry_msgs::Point>(4, geometry_msgs::Point());
        p[0].x = msg->p1.x;
        p[0].y = msg->p1.y;
        p[0].z = msg->p1.z;
        p[1].x = msg->p2.x;
        p[1].y = msg->p2.y;
        p[1].z = msg->p2.z;
        p[2].x = msg->p3.x;
        p[2].y = msg->p3.y;
        p[2].z = msg->p3.z;
        p[3].x = msg->p4.x;
        p[3].y = msg->p4.y;
        p[3].z = msg->p4.z;
        parkingSpace->SetParams(msg->length,
                                msg->width,
                                msg->yaw,
                                p);
        receiveTarget = true;
        std::cout << "receive target info!" << std::endl;
    }
};