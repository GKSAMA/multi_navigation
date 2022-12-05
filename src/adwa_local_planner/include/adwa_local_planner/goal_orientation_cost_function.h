#ifndef GOAL_ORIENTATION_COST_FUNCTION_H
#define GOAL_ORIENTATION_COST_FUNCTION_H

#include <base_local_planner/trajectory_cost_function.h>
#include <geometry_msgs/PoseStamped.h>

namespace base_local_planner
{

  class GoalOrientationCostFunction : public base_local_planner::TrajectoryCostFunction
  {
    public:

      /**
       * @brief Constructor
       */
      GoalOrientationCostFunction(double distance_scale,
        double orientation_scale, double xy_goal_tolerance):
          distance_scale_(distance_scale),
          orientation_scale_(orientation_scale),
          xy_goal_tolerance_(xy_goal_tolerance) {}

      /**
       * @brief Destructor
       */
      ~GoalOrientationCostFunction(){}

      /**
       * @brief Score trajectories based on final orientation correction
       * @param traj: The trajectory to be evaluated
       */
      double scoreTrajectory(base_local_planner::Trajectory& traj);

      /**
       * @brief Used to update context values (not used, just returns true)
       */
      bool prepare();

      /**
       * @brief Set target pose
       * @param goal_pose: The goal pose
       */
      void setTargetPoses(geometry_msgs::PoseStamped goal_pose);

      /**
       * @brief Set distance scale
       * @param distance_scale: The distance scale
       */
      void setDistanceScale(double distance_scale);

      /**
       * @brief Set orientation scale
       * @param orientation_scale: the orientation scale
       */
      void setOrientationScale(double orientation_scale);

      /**
       * @brief Set xy_goal_tolerance
       * @param xy_goal_tolerance: The xy goal tolerance
       */
      void setXYGoalTolerance(double xy_goal_tolerance);

    private:
      //!< goal pose
      geometry_msgs::PoseStamped goal_pose_;

      //!< distance scale for cost function
      double distance_scale_;

      //!< orientation scale for cost function
      double orientation_scale_;

      //!< xy goal tolerance
      double xy_goal_tolerance_;
  };

}

#endif  // GOAL_ORIENTATION_COST_FUNCTION_H