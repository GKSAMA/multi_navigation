/*
 * @Author: Gong, Ke gk.kingj@gmail.com
 * @Date: 2022-12-17 19:27:23
 * @LastEditors: Gong, Ke gk.kingj@gmail.com
 * @LastEditTime: 2023-02-16 17:27:18
 * @FilePath: /src/smooth_global_planner/test/path_smoother_test.cpp
 * @Description: path smoother test
 * 
 * Copyright (c) 2022 by Gong, Ke gk.kingj@gmail.com, All Rights Reserved. 
 */
#include <iostream>
#include <vector>
#include <fstream>
#include <ros/ros.h>
#include <smooth_global_planner/path_smoother.h>
#include <smooth_global_planner/smoother_config_loader.h>
#include <utils/utils.h>
#include <glog/logging.h>
#include "proto/math/smoother_config.h"
#include "time.h"


int main()
{
    std::vector<std::pair<double, double>> raw_point2d;
    std::ifstream data;
    data.open("/home/gk/Documents/multi_turtlebot3_navigation/src/smooth_global_planner/test/path_point.txt", std::ios::out | std::ios::in);
    double x,y;
    while(data >> x >> y){
        raw_point2d.push_back(std::make_pair(x,y));
    }
    // for(auto p : raw_point2d){
    //     std::cout << p.first << " " << p.second << std::endl;
    // }
    data.close();

    SmootherConfig smootherConfig;
    LOG(INFO) << "smootherConfig.weight_fem_pos_deviation = " << smootherConfig.weight_fem_pos_deviation();
    LOG(INFO) << "smootherConfig.weight_path_length = " << smootherConfig.weight_path_length();
    SmootherConfigLoader loader;
    loader.LoadFromFile("/home/gk/Documents/multi_turtlebot3_navigation/src/smooth_global_planner/test/smoother_config_param.yaml", smootherConfig);
    smooth_global_planner::PathSmoother pathSmoother(smootherConfig);
    // LOG(INFO) << "smootherConfig.weight_fem_pos_deviation = " << smootherConfig.weight_fem_pos_deviation();
    // LOG(INFO) << "smootherConfig.weight_path_length = " << smootherConfig.weight_path_length();
    int n = raw_point2d.size();
    std::vector<double> opt_x1, opt_y1;
    std::vector<double> opt_x2, opt_y2;

    std::vector<double> default_bound(n, 7);

    clock_t start,finish;

    bool solve_result1 = false;
    bool solve_result2 = false;
    start = clock();
    solve_result1 = pathSmoother.Solve(raw_point2d, default_bound, &opt_x1, &opt_y1);
    smootherConfig.set_apply_curvature_constraint(true);
    smootherConfig.set_use_sqp(true);
    pathSmoother.SetSmootherConfig(smootherConfig);
    solve_result2 = pathSmoother.Solve(raw_point2d, default_bound, &opt_x2, &opt_y2);
    finish = clock();
    std::cout << "optimize cost time: " << (double)(finish - start) / CLOCKS_PER_SEC << std:: endl;
    if(solve_result1 && solve_result2){
        std::cout << "have a solve result" << std::endl;
        // int nx = opt_x1.size();
        // std::cout << nx << std::endl;

        plot::Figure figure;
        figure.DrawLine(raw_point2d, "origin", plot::color::BLUE);
        // figure.DrawLine(opt_x1, opt_y1, "no curvature", plot::line::DASHED + plot::color::RED);
        figure.DrawLine(opt_x2, opt_y2, "smooth", plot::line::DASHDOT + plot::color::RED);
        // figure.DrawCompare(raw_point2d,"b",opt_x, opt_y, "r");
        figure.PlotShow();
    } else {
        std::cout << "cant solve" << std::endl;
    }
    

}