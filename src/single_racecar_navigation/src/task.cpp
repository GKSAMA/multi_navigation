/*
 * @Author: Gong, Ke gk.kingj@gmail.com
 * @Date: 2022-11-22 21:16:50
 * @LastEditors: Gong, Ke gk.kingj@gmail.com
 * @LastEditTime: 2023-03-31 23:08:54
 * @FilePath: /src/single_racecar_navigation/src/task.cpp
 * @Description: 
 * 
 * Copyright (c) 2022 by Gong, Ke gk.kingj@gmail.com, All Rights Reserved. 
 */
#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <sys/wait.h>
#include <tf2/utils.h>
#include <tf2/convert.h>
#include <geometry_msgs/PoseStamped.h>
#include <parking_task/parking.h>
#include "single_racecar_navigation/gazebo_controller_manager.h"
#include "single_racecar_navigation/load_parameters.h"
#include "single_racecar_navigation/utils.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "parking_task");
    ros::NodeHandle nh;
    std::string pathToPackage = ros::package::getPath("single_racecar_navigation");
    SingleNavUtils::Configurations config(pathToPackage);
    float posx,posy,posz,posth;
    float vx = -0.3, vth = 0.785;
    std::string robotName="smart_0";
    nh.getParam("pos_x", posx);
    nh.getParam("pos_y", posy);
    nh.getParam("pos_z", posz);
    nh.getParam("pos_yaw", posth);
    nh.getParam("robotName", robotName);
    nh.getParam("vel_x", vx);
    nh.getParam("vel_th", vth);
    std::cout << "pos_x=" << posx << " pos_y=" << posy << " pos_z=" << posz << " pos_yaw=" << posth << std::endl;
    // ResetRobotBySystem(nh, robotName, posx, posy, posz, posth);
    // sleep(1);
    // std::ifstream ifspace;
    // ifspace.open("../config/parking.data",ios::in,0);
    std::vector<geometry_msgs::Point> points = std::vector<geometry_msgs::Point>(4, geometry_msgs::Point());
    // ifspace >> points[0].x >> points[0].y >> points[1].x >> points[1].y >> points[2].x >> points[2].y >> points[3].x >> points[3].y;
    points[0].x = 2.3;
    points[0].y = 4.7;
    points[1].x = 4.8;
    points[1].y = 4.7;
    points[2].x = 4.8;
    points[2].y = 0.0;
    points[3].x = 2.3;
    points[3].y = 0.0;

    ParkingTask ptask(nh,robotName);
    // ParkingSpace ps(4.7, 2.5, M_PI / 2, points);
    ParkingSpace ps(6, 2.5, M_PI / 2, points);
    // CarParam cp(0.46, 0.38, 0.335, 0.20, M_PI / 4);
    CarParam cp(2.695, 1.449, 1.868, 1.284, M_PI / 4);
    geometry_msgs::PoseStamped *carState = new geometry_msgs::PoseStamped();
    carState->pose.position.x = posx;
    carState->pose.position.y = posy;
    carState->pose.position.z = posz;
    tf2::Quaternion q;
    q.setRPY(0,0,posth);
    tf2::convert(q, carState->pose.orientation);
    ptask.SetCarState(carState);
    // ptask.SetParkingSpace(ps);
    // ptask.SetCarParam(cp);
    // ptask.SetSafeDistance(-1);
    ptask.SetParkingSpace(config.GetParkingSpace());
    ptask.SetCarParam(config.GetCarParam());
    ptask.SetSafeDistance(config.GetSafeDist());
    // bool flag = ptask.VerticalTask(vx);
    bool flag = ptask.ParallelTask(vx);
    if(flag){
        std::cout << "succeed !" << std::endl;
    } else {
        std::cout << "failed !" << std::endl;
    }
    // ptask.test(-vx, 0.785);
    ros::shutdown();
    return 0;
}