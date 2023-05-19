/*
 * @Author: Gong, Ke gk.kingj@gmail.com
 * @Date: 2022-12-19 14:17:30
 * @LastEditors: Gong, Ke gk.kingj@gmail.com
 * @LastEditTime: 2022-12-19 14:56:40
 * @FilePath: /src/smooth_global_planner/src/smoother_config_loader.cpp
 * @Description: 
 * 
 * Copyright (c) 2022 by Gong, Ke gk.kingj@gmail.com, All Rights Reserved. 
 */
#include "smooth_global_planner/smoother_config_loader.h"

SmootherConfigLoader::SmootherConfigLoader()
{

}



void SmootherConfigLoader::LoadFromFile(const std::string &filename, 
                                        SmootherConfig& smootherConfig)
{
    std::cout << "load config params from " << filename << std::endl;
    YAML::Node config = YAML::LoadFile(filename);
    float weight_fem_pos_deviation = config["SmootherConfig"]["weight_fem_pos_deviation"].as<float>();
    // std::cout << "weight_fem_pos_deviation = " << weight_fem_pos_deviation << std::endl;
    float weight_ref_deviation = config["SmootherConfig"]["weight_ref_deviation"].as<float>();
    // std::cout << "weight_ref_deviation = " << weight_ref_deviation << std::endl;
    float weight_path_length = config["SmootherConfig"]["weight_path_length"].as<float>();
    // std::cout << "weight_path_length = " << weight_path_length << std::endl;
    bool apply_curvature_constraint = config["SmootherConfig"]["apply_curvature_constraint"].as<bool>();
    // std::cout << "apply_curvature_constraint = " << apply_curvature_constraint << std::endl;
    float weight_curvature_constraint_slack_var = config["SmootherConfig"]["weight_curvature_constraint_slack_var"].as<float>();
    // std::cout << "weight_curvature_constraint_slack_var = " << weight_curvature_constraint_slack_var << std::endl;
    float curvature_constraint = config["SmootherConfig"]["curvature_constraint"].as<float>();
    // std::cout << "curvature_constraint = " << curvature_constraint << std::endl;
    bool use_sqp = config["SmootherConfig"]["use_sqp"].as<bool>();
    // std::cout << "use_sqp = " << use_sqp << std::endl;
    float sqp_ftol = config["SmootherConfig"]["sqp_ftol"].as<float>();
    // std::cout << "sqp_ftol = " << sqp_ftol << std::endl;
    float sqp_ctol = config["SmootherConfig"]["sqp_ctol"].as<float>();
    // std::cout << "sqp_ctol = " << sqp_ctol << std::endl;
    int sqp_pen_max_iter = config["SmootherConfig"]["sqp_pen_max_iter"].as<int>();
    // std::cout << "sqp_pen_max_iter = " << sqp_pen_max_iter << std::endl;
    int sqp_sub_max_iter = config["SmootherConfig"]["sqp_sub_max_iter"].as<int>();
    // std::cout << "sqp_sub_max_iter = " << sqp_sub_max_iter << std::endl;
    int max_iter = config["SmootherConfig"]["max_iter"].as<int>();
    // std::cout << "max_iter = " << max_iter << std::endl;
    float time_limit = config["SmootherConfig"]["time_limit"].as<float>();
    // std::cout << "time_limit = " << time_limit << std::endl;
    bool verbose = config["SmootherConfig"]["verbose"].as<bool>();
    // std::cout << "verbose = " << verbose << std::endl;
    bool scaled_termination = config["SmootherConfig"]["scaled_termination"].as<bool>();
    // std::cout << "scaled_termination = " << scaled_termination << std::endl;
    bool warm_start = config["SmootherConfig"]["warm_start"].as<bool>();
    // std::cout << "warm_start = " << warm_start << std::endl;
    int print_level = config["SmootherConfig"]["print_level"].as<int>();
    // std::cout << "print_level = " << print_level << std::endl;
    float max_num_of_iterations = config["SmootherConfig"]["max_num_of_iterations"].as<int>();
    // std::cout << "max_num_of_iterations = " << max_num_of_iterations << std::endl;
    int acceptable_num_of_iterations = config["SmootherConfig"]["acceptable_num_of_iterations"].as<int>();
    // std::cout << "acceptable_num_of_iterations = " << acceptable_num_of_iterations << std::endl;
    float tol = config["SmootherConfig"]["tol"].as<float>();
    // std::cout << "tol = " << tol << std::endl;
    float acceptable_tol = config["SmootherConfig"]["acceptable_tol"].as<float>();
    // std::cout << "acceptable_tol = " << acceptable_tol << std::endl;


    smootherConfig.set_weight_fem_pos_deviation(weight_fem_pos_deviation);
    smootherConfig.set_weight_ref_deviation(weight_ref_deviation);
    smootherConfig.set_weight_path_length(weight_path_length);
    smootherConfig.set_apply_curvature_constraint(apply_curvature_constraint);
    smootherConfig.set_weight_curvature_constraint_slack_var(weight_curvature_constraint_slack_var);
    smootherConfig.set_curvature_constraint(curvature_constraint);
    smootherConfig.set_use_sqp(use_sqp);
    smootherConfig.set_sqp_ftol(sqp_ftol);
    smootherConfig.set_sqp_ctol(sqp_ctol);
    smootherConfig.set_sqp_pen_max_iter(sqp_pen_max_iter);
    smootherConfig.set_sqp_sub_max_iter(sqp_sub_max_iter);
    smootherConfig.set_max_iter(max_iter);
    smootherConfig.set_time_limit(time_limit);
    smootherConfig.set_verbose(verbose);
    smootherConfig.set_scaled_termination(scaled_termination);
    smootherConfig.set_warm_start(warm_start);
    smootherConfig.set_print_level(print_level);
    smootherConfig.set_max_num_of_iterations(max_num_of_iterations);
    smootherConfig.set_acceptable_num_of_iterations(acceptable_num_of_iterations);
    smootherConfig.set_tol(tol);
    smootherConfig.set_acceptable_tol(acceptable_tol);

    std::cout << "load finished ..."  << std::endl;
}