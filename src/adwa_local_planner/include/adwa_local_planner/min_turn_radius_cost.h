#ifndef MIN_TURN_RADIUS_COST_H
#define MIN_TURN_RADIUS_COST_H

#include <base_local_planner/trajectory_cost_function.h>
#include <base_local_planner/map_grid.h>
#include <costmap_2d/costmap_2d.h>
#include "tf/transform_listener.h"
#include "tf2/convert.h"
#include "tf2/transform_datatypes.h"
#include "tf2/utils.h"

namespace base_local_planner{

    class MinTurnRadiusCostFunction : public base_local_planner::TrajectoryCostFunction{
    public:
        MinTurnRadiusCostFunction();
        ~MinTurnRadiusCostFunction() {}

        bool prepare();
        double scoreTrajectory(Trajectory &traj);
        void setScale(double scale);
        void setAngularVelocity(double cur_angular_velocity);
        void setFixedParams(double max_angular_velocity, double wheel_base);

        void setTargetPose(geometry_msgs::PoseStamped pose);

    private:
        geometry_msgs::PoseStamped target_pose;
        double maxAngularVelocity;
        double minTurnRadius;
        double curAngularVelocity;
        double wheelbase;
        double turnCostScale;
    };
}

#endif //VERTICAL_DISTANCE_COST_H