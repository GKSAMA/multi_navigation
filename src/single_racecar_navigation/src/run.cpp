/*
 * @Author: Gong, Ke gk.kingj@gmail.com
 * @Date: 2022-11-14 13:38:44
 * @LastEditors: Gong, Ke gk.kingj@gmail.com
 * @LastEditTime: 2023-05-03 15:02:22
 * @FilePath: /src/single_racecar_navigation/src/run.cpp
 * @Description: 
 * 
 * Copyright (c) 2022 by Gong, Ke gk.kingj@gmail.com, All Rights Reserved. 
 */
#include<iostream>
#include<ros/ros.h>
#include<sys/wait.h>
#include<string>

int main()
{
    pid_t gazebo_pid = fork();
    if(gazebo_pid == 0){
        system(std::string("roslaunch single_racecar_navigation gazebo_racecar.launch").c_str());
        // system(std::string("roslaunch single_racecar_navigation gazebo_smart.launch").c_str());
        // system(std::string("roslaunch single_racecar_navigation gazebo_f1tenth.launch").c_str());
        exit(0);
    }
    sleep(5);
    pid_t movebase_pid = -1;
    if(gazebo_pid != 0){
        movebase_pid = fork();
    }
    if(movebase_pid == 0){
        system(std::string("roslaunch single_racecar_navigation move_base_racecar.launch").c_str());
        // system(std::string("roslaunch single_racecar_navigation move_base_smart.launch").c_str());
        // system(std::string("roslaunch single_racecar_navigation move_base_f1tenth.launch").c_str());
        exit(0);
    }
    sleep(3);
    system(std::string("roslaunch single_racecar_navigation navigation_racecar.launch").c_str());
    // system(std::string("roslaunch single_racecar_navigation navigation_smart.launch").c_str());
    // system(std::string("roslaunch single_racecar_navigation navigation_f1tenth.launch").c_str());
    // system(std::string("roslaunch single_racecar_navigation navigation_smart_test.launch").c_str());
    int status = 0;
    waitpid(gazebo_pid, &status, 0);
    waitpid(movebase_pid, &status, 0);
    
    return 0;
}