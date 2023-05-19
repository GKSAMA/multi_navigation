/*
 * @Author: Gong, Ke gk.kingj@gmail.com
 * @Date: 2022-12-16 17:18:56
 * @LastEditors: Gong, Ke gk.kingj@gmail.com
 * @LastEditTime: 2022-12-18 17:16:12
 * @FilePath: /src/smooth_global_planner/include/smooth_global_planner/path_smoother.h
 * @Description: 
 * 
 * Copyright (c) 2022 by Gong, Ke gk.kingj@gmail.com, All Rights Reserved. 
 */
#ifndef SMOOTH_GLOBAL_PLANNER_PATH_SMOOTHER_
#define SMOOTH_GLOBAL_PLANNER_PATH_SMOOTHER_

#include <utility>
#include <vector>

#include <geometry_msgs/PoseStamped.h>

#include "proto/math/smoother_config.h"

namespace smooth_global_planner{
    class PathSmoother{
    public:
        PathSmoother();
        explicit PathSmoother(const SmootherConfig& config);

        void SetSmootherConfig(const SmootherConfig& config);

        bool Solve(const std::vector<geometry_msgs::PoseStamped>& path,
                    const std::vector<double>& bounds, std::vector<double>* opt_x,
                    std::vector<double>* opt_y);

        bool Solve(const std::vector<std::pair<double, double>>& raw_point2d,
                    const std::vector<double>& bounds, std::vector<double>* opt_x,
                    std::vector<double>* opt_y);

        bool QpWithOsqp(const std::vector<std::pair<double, double>>& raw_point2d,
                        const std::vector<double>& bounds, std::vector<double>* opt_x,
                        std::vector<double>* opt_y);
        bool SqpWithOsqp(const std::vector<std::pair<double, double>>& raw_point2d,
                        const std::vector<double>& bounds, std::vector<double>* opt_x,
                        std::vector<double>* opt_y);
    private:
        SmootherConfig config_;
    };
};


#endif // SMOOTH_GLOBAL_PLANNER_PATH_SMOOTHER_