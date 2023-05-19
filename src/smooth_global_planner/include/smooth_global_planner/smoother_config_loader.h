/*
 * @Author: Gong, Ke gk.kingj@gmail.com
 * @Date: 2022-12-19 14:17:14
 * @LastEditors: Gong, Ke gk.kingj@gmail.com
 * @LastEditTime: 2022-12-19 15:00:54
 * @FilePath: /src/smooth_global_planner/include/smooth_global_planner/smoother_config_loader.h
 * @Description: 
 * 
 * Copyright (c) 2022 by Gong, Ke gk.kingj@gmail.com, All Rights Reserved. 
 */
#ifndef SMOOTHER_GLOBAL_PLANNER_SMOOTHER_CONFIG_LOADER_
#define SMOOTHER_GLOBAL_PLANNER_SMOOTHER_CONFIG_LOADER_

#include <yaml-cpp/yaml.h>
#include <iostream>
#include <string>
#include "smooth_global_planner/path_smoother.h"
#include "proto/math/smoother_config.h"

class SmootherConfigLoader{
public:
    SmootherConfigLoader();
    void LoadFromFile(const std::string &filename, SmootherConfig& smootherConfig);
};

#endif // SMOOTHER_GLOBAL_PLANNER_SMOOTHER_CONFIG_LOADER_