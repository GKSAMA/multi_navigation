#ifndef VERTICAL_DISTANCE_COST_H
#define VERTICAL_DISTANCE_COST_H

#include <base_local_planner/trajectory_cost_function.h>
#include <base_local_planner/map_grid.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d.h>
#include "tf/transform_listener.h"
#include "tf2/convert.h"
#include "tf2/transform_datatypes.h"
#include "tf2/utils.h"

namespace base_local_planner{
    struct map_inf{
        double size_x;
        double size_y;
        double scale;
        double origin_x;
        double origin_y;
    };

    class VerticalDistanceCostFunction : public base_local_planner::TrajectoryCostFunction{
    public:
        VerticalDistanceCostFunction(){}
        VerticalDistanceCostFunction(costmap_2d::Costmap2D* costmap);
        ~VerticalDistanceCostFunction();

        bool prepare();
        double scoreTrajectory(Trajectory &traj);
        void SetScale(double scale);

        void SetTargetPose(geometry_msgs::PoseStamped pose);

        void SetGlobalPathLength(double len) {global_path_len = len;}

        void ComputeTurnRadius(double wheelbase, double max_steer_angle);

        double ComputeMinObstacleDistance(int x, int y);
    private:
        costmap_2d::Costmap2D* costmap_;
        base_local_planner::WorldModel* world_model_;
        geometry_msgs::PoseStamped target_pose;
        double min_turn_radius_;
        // double min_obstacle_dist_;
        double distance_scale;
        double global_path_len;
    };
}

#endif //VERTICAL_DISTANCE_COST_H