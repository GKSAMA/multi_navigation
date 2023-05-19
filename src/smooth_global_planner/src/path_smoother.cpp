/*
 * @Author: Gong, Ke gk.kingj@gmail.com
 * @Date: 2022-12-17 17:34:11
 * @LastEditors: Gong, Ke gk.kingj@gmail.com
 * @LastEditTime: 2022-12-18 19:37:03
 * @FilePath: /src/smooth_global_planner/src/path_smoother.cpp
 * @Description: 
 * 
 * Copyright (c) 2022 by Gong, Ke gk.kingj@gmail.com, All Rights Reserved. 
 */
#include <iostream>

#include <glog/logging.h>

#include <smooth_global_planner/path_smoother.h>
#include <smooth_global_planner/interface/osqp_interface.h>
#include <smooth_global_planner/interface/sqp_osqp_interface.h>

namespace smooth_global_planner{

PathSmoother::PathSmoother()
{

}

PathSmoother::PathSmoother(const SmootherConfig& config) {
    config_ = config;
}

void PathSmoother::SetSmootherConfig(const SmootherConfig& config)
{
    config_ = config;
}

bool PathSmoother::Solve(const std::vector<geometry_msgs::PoseStamped>& path,
            const std::vector<double>& bounds, std::vector<double>* opt_x,
            std::vector<double>* opt_y){
    std::vector<std::pair<double, double>> raw_point2d;
    for(auto p : path){
        raw_point2d.emplace_back(std::make_pair(p.pose.position.x, p.pose.position.y));
    }
    if(config_.apply_curvature_constraint()){
        if(config_.use_sqp()){
            return SqpWithOsqp(raw_point2d, bounds, opt_x, opt_y);
        }
    }else {
        return QpWithOsqp(raw_point2d, bounds, opt_x, opt_y);
    }
}

bool PathSmoother::Solve(const std::vector<std::pair<double, double>>& raw_point2d,
            const std::vector<double>& bounds, std::vector<double>* opt_x,
            std::vector<double>* opt_y){
    return QpWithOsqp(raw_point2d, bounds, opt_x, opt_y);
}

bool PathSmoother::QpWithOsqp(const std::vector<std::pair<double, double>>& raw_point2d,
                                const std::vector<double>& bounds, std::vector<double>* opt_x,
                                std::vector<double>* opt_y) {
    if (opt_x == nullptr || opt_y == nullptr) {
        std::cout << "opt_x or opt_y is nullptr" << std::endl;
        return false;
    }

    OsqpInterface solver;

    solver.set_weight_fem_pos_deviation(config_.weight_fem_pos_deviation());
    solver.set_weight_path_length(config_.weight_path_length());
    solver.set_weight_ref_deviation(config_.weight_ref_deviation());

    solver.set_max_iter(config_.max_iter());
    solver.set_time_limit(config_.time_limit());
    solver.set_verbose(config_.verbose());
    solver.set_scaled_termination(config_.scaled_termination());
    solver.set_warm_start(config_.warm_start());

    solver.set_ref_points(raw_point2d);
    solver.set_bounds_around_refs(bounds);

    if (!solver.Solve()) {
        return false;
    }

    *opt_x = solver.opt_x();
    *opt_y = solver.opt_y();
    return true;
}

bool PathSmoother::SqpWithOsqp(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds, std::vector<double>* opt_x,
    std::vector<double>* opt_y) {
  if (opt_x == nullptr || opt_y == nullptr) {
    LOG(ERROR) << "opt_x or opt_y is nullptr";
    return false;
  }

  SqpOsqpInterface solver;

  solver.set_weight_fem_pos_deviation(config_.weight_fem_pos_deviation());
  solver.set_weight_path_length(config_.weight_path_length());
  solver.set_weight_ref_deviation(config_.weight_ref_deviation());
  solver.set_weight_curvature_constraint_slack_var(
      config_.weight_curvature_constraint_slack_var());

  solver.set_curvature_constraint(config_.curvature_constraint());

  solver.set_sqp_sub_max_iter(config_.sqp_sub_max_iter());
  solver.set_sqp_ftol(config_.sqp_ftol());
  solver.set_sqp_pen_max_iter(config_.sqp_pen_max_iter());
  solver.set_sqp_ctol(config_.sqp_ctol());

  solver.set_max_iter(config_.max_iter());
  solver.set_time_limit(config_.time_limit());
  solver.set_verbose(config_.verbose());
  solver.set_scaled_termination(config_.scaled_termination());
  solver.set_warm_start(config_.warm_start());

  solver.set_ref_points(raw_point2d);
  solver.set_bounds_around_refs(bounds);

  if (!solver.Solve()) {
    return false;
  }

  std::vector<std::pair<double, double>> opt_xy = solver.opt_xy();

  // TODO(Jinyun): unify output data container
  opt_x->resize(opt_xy.size());
  opt_y->resize(opt_xy.size());
  for (size_t i = 0; i < opt_xy.size(); ++i) {
    (*opt_x)[i] = opt_xy[i].first;
    (*opt_y)[i] = opt_xy[i].second;
  }
  return true;
}
}; // namespace smooth_global_planner