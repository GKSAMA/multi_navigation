#ifndef CROSS_POINT_COST_H
#define CROSS_POINT_COST_H

#include <base_local_planner/trajectory_cost_function.h>
#include <base_local_planner/map_grid.h>
#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/costmap_model.h>
#include "tf/transform_listener.h"
#include "tf2/convert.h"
#include "tf2/transform_datatypes.h"
#include "tf2/utils.h"


namespace{
    class CrossPointCostFunction : public base_local_planner::TrajectoryCostFunction{
    public:
        CrossPointCostFunction();
        ~CrossPointCostFunction() {}

        bool prepare();
        double scoreTrajectory(Trajectory &traj);

        void SetTargetPose(geometry_msgs::PoseStamped pose) { target_pose_ = pose; }
        void SetScale(double scale) { crossPointScale_ = scale; }

        std::pair<double,double> ComputeCrossPoint(double x1,
                                                    double y1,
                                                    double th1,
                                                    double x2,
                                                    double y2,
                                                    double th2);
    private:
        geometry_msgs::PoseStamped target_pose_;
        double crossPointScale_;
    };
}

#endif // CROSS_POINT_COST_H