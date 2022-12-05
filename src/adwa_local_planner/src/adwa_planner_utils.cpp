#include "adwa_local_planner/adwa_planner_utils.h"

#include <base_local_planner/goal_functions.h>

AdwaPlannerUtil::AdwaPlannerUtil():initialized_(false)
{

}

AdwaPlannerUtil::~AdwaPlannerUtil()
{

}

void AdwaPlannerUtil::initialize(tf2_ros::Buffer* tf, costmap_2d::Costmap2D* costmap, std::string global_frame)
{
    if(!initialized_){
        tf_ = tf;
        costmap_ = costmap;
        global_frame_ = global_frame;
        initialized_ = true;
    } else {
        ROS_WARN("Planner utils have already been initialized, doing nothing.");
    }
}

void AdwaPlannerUtil::reconfigureCB(AdwaLimits& config, bool restore_defaults)
{
    if(setup_ && restore_defaults){
        config = default_limits_;
    }
    if(!setup_){
        default_limits_ = config;
        setup_ = true;
    }
    std::lock_guard<std::mutex> limitGuard(limits_configuration_mutex_);
    limits_ = AdwaLimits(config);
    // std::cout << limits_;
}

costmap_2d::Costmap2D* AdwaPlannerUtil::getCostmap() 
{
  return costmap_;
}

AdwaLimits AdwaPlannerUtil::getCurrentLimits() 
{
  std::lock_guard<std::mutex> limitGuard(limits_configuration_mutex_);
  return limits_;
}

std::string AdwaPlannerUtil::getGlobalFrame()
{
  return global_frame_;
}

bool AdwaPlannerUtil::getGoal(geometry_msgs::PoseStamped& goal_pose) 
{
  //we assume the global goal is the last point in the global plan
  return base_local_planner::getGoalPose(*tf_, global_plan_, global_frame_, goal_pose);
}

bool AdwaPlannerUtil::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    if(!initialized_) {
        ROS_ERROR("Planner utils have not been initialized, please call initialize() first");
        return false;
    }
    ////divide plan by manuveurs
    subPathList.clear();
    subPath.clear();
    subPathIndex = 0;
    double pathLength = 0;
    for(unsigned int i = 0; i < orig_global_plan.size(); ++i) {
        if(i > 1 && i < orig_global_plan.size()) {
            double x0 = orig_global_plan[i].pose.position.x;
            double x1 = orig_global_plan[i - 1].pose.position.x;
            double x2 = orig_global_plan[i - 2].pose.position.x;
            double y0 = orig_global_plan[i].pose.position.y;
            double y1 = orig_global_plan[i - 1].pose.position.y;
            double y2 = orig_global_plan[i - 2].pose.position.y;
            // cos(a + b)
            double angle=((x0 - x1) * (x1 - x2) + (y0 - y1) * (y1 - y2)) / 
                        (std::sqrt(std::pow(x0 - x1, 2) + std::pow(y0 - y1, 2)) * std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2)));
            if(angle < -1.0)
                angle = -1.0;
            else if(angle > 1.0)
                angle = 1.0;
            angle = std::acos(angle);
            pathLength += std::sqrt(std::pow(x0 - x1, 2) + std::pow(y0 - y1, 2));
            //if changes of direction detected 
            if(fabs(angle) > 1.57) {
                if(pathLength > 1.0) {
                    // ROS_INFO("TrajectoryPlannerROS::setPlan: subPath split at i=%d, angle=%f, length=%f", i, angle, pathLength);
                    subPathList.emplace_back(subPath);
                } else {//ignored subpaths shorter than 1.0m
                    // ROS_INFO("TrajectoryPlannerROS::setPlan: subPath split at i=%d, angle=%f, Ignored by length=%f", i, angle, pathLength);
                }
                subPath.clear();
                pathLength = 0.0;
            }
        }
        subPath.emplace_back(orig_global_plan[i]);
    }
    subPathList.emplace_back(subPath);
    // ROS_INFO("TrajectoryPlannerROS::setPlan: subPath last length=%f", pathLength);
    // ROS_INFO("TrajectoryPlannerROS::setPlan: Global plan (%lu points) split in %lu paths", orig_global_plan.size(), subPathList.size());

    //reset the global plan
    global_plan_.clear(); 

    global_plan_ = subPathList[subPathIndex];
    // ROS_INFO("set Plan successfully!");

    return true;
}

bool AdwaPlannerUtil::setNextPath()
{
    ////check if there are manuveurs remaining
    if(subPathIndex < subPathList.size() - 1) {
        subPathIndex++;
        global_plan_.clear();
        global_plan_ = subPathList[subPathIndex];
        return true;
    } else {
        return false;
    }
}

bool AdwaPlannerUtil::lastPath()
{
    if(subPathIndex == subPathList.size()-1){
        return true;
    } else {
        return false;
    }
}

bool AdwaPlannerUtil::getLocalPlan(geometry_msgs::PoseStamped& global_pose, std::vector<geometry_msgs::PoseStamped>& transformed_plan) 
{
    global_pose.header.stamp -= ros::Duration(1.0);

    //get the global plan in our frame
    if(!base_local_planner::transformGlobalPlan(*tf_,global_plan_,global_pose,*costmap_,global_frame_,transformed_plan)) {
        ROS_WARN("Could not transform the global plan to the frame of the controller");
        return false;
    }

    //now we'll prune the plan based on the position of the robot
    if(limits_.prune_plan) {
        base_local_planner::prunePlan(global_pose, transformed_plan, global_plan_);
    }

    return true;
}