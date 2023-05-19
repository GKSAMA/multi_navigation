#ifndef HEADING_WITH_PATH
#define HEADING_WITH_PATH

#include <base_local_planner/trajectory_cost_function.h>
#include <base_local_planner/map_grid.h>
#include <costmap_2d/costmap_2d.h>
#include "tf/transform_listener.h"
#include "tf2/convert.h"
#include "tf2/transform_datatypes.h"
#include "tf2/utils.h"

#include "cost_state.h"

namespace base_local_planner{
    class HeadingWithPathCostFunction : public base_local_planner::TrajectoryCostFunction{
    public:
        HeadingWithPathCostFunction();
        ~HeadingWithPathCostFunction(){}

        bool prepare() { return true; }
        double scoreTrajectory(Trajectory& traj);

        void SetTargetPose(geometry_msgs::PoseStamped pose) { target_pose_ = pose;}
        void SetScale(double scale) { heading_scale_ = scale; }

    private:
        geometry_msgs::PoseStamped target_pose_;
        CostState* costState_;
        double heading_scale_;
    };
}