#ifndef HEADING_COST_FUNCTION_H
#define HEADING_COST_FUNCTION_H

#include <base_local_planner/trajectory_cost_function.h>
#include <geometry_msgs/PoseStamped.h>

class AdwaHeadingCostFunction: public base_local_planner::TrajectoryCostFunction
{
private:
    /* data */
public:
    AdwaHeadingCostFunction();
    ~AdwaHeadingCostFunction();

    bool prepare();
    double scoreTrajectory(base_local_planner::Trajectory& traj);
    void setGlobalPlan(std::vector<geometry_msgs::PoseStamped> &traj);
    void setScale(double scale);

private:
    std::vector<geometry_msgs::PoseStamped> global_plan;
    geometry_msgs::PoseStamped goal;
    double scale;
};

#endif // HEADING_COST_FUNCTION_H