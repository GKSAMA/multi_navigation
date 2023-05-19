/*
 * @Author: Gong, Ke gk.kingj@gmail.com
 * @Date: 2022-11-23 13:45:03
 * @LastEditors: Gong, Ke gk.kingj@gmail.com
 * @LastEditTime: 2022-11-23 14:12:46
 * @FilePath: /src/single_racecar_navigation/include/single_racecar_navigation/gazebo_controller_manager.h
 * @Description: 
 * 
 * Copyright (c) 2022 by Gong, Ke gk.kingj@gmail.com, All Rights Reserved. 
 */
#ifndef GAZEBO_CONTROLLER_MANAGER_H
#define GAZEBO_CONTROLLER_MANAGER_H

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <gazebo_msgs/SetPhysicsProperties.h>
#include <std_srvs/Empty.h>

#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/ListControllerTypes.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/ReloadControllerLibraries.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ControllerState.h>
#include <chrono>


/**
 * @brief init simulation controllers
 * @param nodeHandle
 * @param serviceName
 */
bool startControllers(ros::NodeHandle &nodeHandle, std::string serviceName);

/**
 * @brief stop simulation controllers
 * @param nodeHandle
 * @param serviceName
 */
bool stopControllers(ros::NodeHandle &nodeHandle, std::string serviceName);

/**
 * @brief restart simultion by system command
 * 
 */
bool ResetRobotBySystem(ros::NodeHandle &nodeHandle, std::string robotName, float pos_x, float pos_y, float pos_z, float pos_th);

/**
 * @brief restart simulation by gazebo-ros service
 * @param modelStateClient
 * @param jointStateClient
 */
bool ResetRobotByService(ros::NodeHandle &nodeHandle,
                         ros::ServiceClient &modelStateClient, 
                         ros::ServiceClient &jointStateClient, 
                         std::string robotName);


#endif //GAZEBO_CONTROLLER_MANAGER_H