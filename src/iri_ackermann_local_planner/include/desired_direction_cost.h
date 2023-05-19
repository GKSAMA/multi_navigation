
#ifndef DESIRED_DIRECTION_COST_H
#define DESIRED_DIRECTION_COST_H

#include <base_local_planner/trajectory_cost_function.h>
#include <base_local_planner/map_grid.h>
#include <costmap_2d/costmap_2d.h>
#include "tf/transform_listener.h"
#include "tf2/convert.h"
#include "tf2/transform_datatypes.h"
#include "tf2/utils.h"

#include "cost_state.h"

namespace base_local_planner{

    class DesiredDirectionCostFunction : public base_local_planner::TrajectoryCostFunction{
    public:
        DesiredDirectionCostFunction();
        ~DesiredDirectionCostFunction() {}

        bool prepare();
        double scoreTrajectory(Trajectory &traj);

        void SetTargetPose(geometry_msgs::PoseStamped pose);
        void SetScale(double scale);
        double getScale() {return direction_scale;}

    private:
        geometry_msgs::PoseStamped target_pose;
        CostState *costState_;
        double direction_scale;
    };
}

#endif // DESIRED_DIRECTION_COST_H