/*
 * @Author: Gong, Ke gk.kingj@gmail.com
 * @Date: 2022-12-20 10:56:59
 * @LastEditors: Gong, Ke gk.kingj@gmail.com
 * @LastEditTime: 2023-02-16 16:46:33
 * @FilePath: /src/smooth_global_planner/src/utils/utils.cpp
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#include "utils/utils.h"

#include "third_party/matplotlibcpp.h"

namespace plot{
    Figure::Figure(){
        high_ = 1280;
        width_ = 720;
        matplotlibcpp::figure_size(high_, width_);
    }

    Figure::Figure(int h, int w):high_(h), width_(w)
    {
        matplotlibcpp::figure_size(high_, width_);
    }

    void Figure::DrawLine(const std::vector<std::pair<double, double>> &points, const std::string &name, const std::string color){
        int n = points.size();
        std::vector<double> x(n);
        std::vector<double> y(n);
        for(int i = 0; i < n; ++i){
            x[i] = points[i].first;
            y[i] = points[i].second;
        }
        matplotlibcpp::named_plot(name, x, y, color);
        matplotlibcpp::xlabel("x");
        matplotlibcpp::ylabel("y");
    }

    void Figure::DrawLine(const std::vector<double> &x, const std::vector<double> &y, const std::string &name, std::string color){
        matplotlibcpp::named_plot(name, x, y, color);
        matplotlibcpp::xlabel("x");
        matplotlibcpp::ylabel("y");
    }

    void Figure::DrawCompare(const std::vector<std::pair<double, double>> &points, 
                    std::string color1,
                    const std::vector<double> &x2, const std::vector<double> &y2, 
                    std::string color2){
        int n = points.size();
        std::vector<double> x1(n);
        std::vector<double> y1(n);
        for(int i = 0; i < n; ++i){
            x1[i] = points[i].first;
            y1[i] = points[i].second;
        }
        matplotlibcpp::named_plot("origin",x1,y1,color1);
        matplotlibcpp::named_plot("modify",x2,y2,color2);
        matplotlibcpp::xlabel("x");
        matplotlibcpp::ylabel("y");
    }

    void Figure::PlotShow(){
        matplotlibcpp::legend();
        matplotlibcpp::show();
    }
}