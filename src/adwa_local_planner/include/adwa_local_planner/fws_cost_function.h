#ifndef FWS_COST_FUNCTION_H
#define FWS_COST_FUNCTION_H

#include <base_local_planner/trajectory_cost_function.h>

namespace base_local_planner
{

  class FWSCostFunction : public base_local_planner::TrajectoryCostFunction
  {
    public:

      /**
       * @brief Constructor
       * @param r_min: minimum turning radius of the robot
       * @param b_max: maximum diagonal motion angle
       */
      FWSCostFunction(double r_min, double b_max) : r_min_(r_min), b_max_(b_max)
      {
      }

      /**
       * @brief Destructor
       */
      ~FWSCostFunction()
      {
      }

      /**
       * @brief Punishes inadmissible trajectories based on FWS constraints
       * @param traj: The trajectory to be evaluated
       */
      double scoreTrajectory(base_local_planner::Trajectory& traj);

      /**
       * @brief Used to update context values (not used, just returns true)
       */
      bool prepare() {return true;}

      /**
       * @brief Set minimum turning radius
       */
      void setRMin(double r_min) {r_min_ = r_min;}

      /**
       * @brief Set maximum diagonal motion angle
       */
      void setBMax(double b_max) {b_max_ = b_max;}

    private:
      //!< minimum turning radius of the robot
      double r_min_;

      //!< maximum diagonal motion angle
      double b_max_;
  };

}

#endif  // FWS_COST_FUNCTION_H