#include "adwa_local_planner/goal_orientation_cost_function.h"
#include <math.h>
#include <tf/tf.h>

namespace base_local_planner
{

  double GoalOrientationCostFunction::scoreTrajectory(base_local_planner::Trajectory& traj)
  {
    double cx, cy, cth;  // current pose
    double nx, ny, nth;  // new pose
    double gx, gy, gth;  // goal pose

    traj.getPoint(0, cx, cy, cth);  // get current pose
    traj.getPoint(traj.getPointsSize() - 1, nx, ny, nth);  // get new pose

    // get goal pose
    gx = goal_pose_.pose.position.x;
    gy = goal_pose_.pose.position.y;
    gth = tf::getYaw(goal_pose_.pose.orientation);

    double cg_dist = hypot(cx-gx, cy-gy);  // current distance from goal
    double ng_dist = hypot(nx-gx, ny-gy);  // new distance from goal

    double cg_dth = fabs(cth - gth);  // current orientation error
    double ng_dth = fabs(nth - gth);  // new orientation error

    double score =
      (cg_dist > 2 * xy_goal_tolerance_)
      * (distance_scale_ * ng_dist + orientation_scale_ * (ng_dth));

    return (score > 0) ? score : 0;
  }


  void GoalOrientationCostFunction::setTargetPoses(
    geometry_msgs::PoseStamped goal_pose)
  {
    goal_pose_ = goal_pose;
  }


  bool GoalOrientationCostFunction::prepare()
  {
    return true;
  }


  void GoalOrientationCostFunction::setDistanceScale(double distance_scale)
  {
    distance_scale_ = distance_scale;
  }


  void GoalOrientationCostFunction::setOrientationScale(double orientation_scale)
  {
    orientation_scale_ = orientation_scale;
  }


  void GoalOrientationCostFunction::setXYGoalTolerance(double xy_goal_tolerance)
  {
    xy_goal_tolerance_ = xy_goal_tolerance;
  }

}