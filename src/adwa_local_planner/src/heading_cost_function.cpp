#include "adwa_local_planner/heading_cost_function.h"
#include <math.h>
#include <limits.h>
#include <values.h>
#include <tf2/utils.h>
#include <angles/angles.h>

AdwaHeadingCostFunction::AdwaHeadingCostFunction()
{
}

AdwaHeadingCostFunction::~AdwaHeadingCostFunction()
{

}

bool AdwaHeadingCostFunction::prepare()
{
    return true;
}

double AdwaHeadingCostFunction::scoreTrajectory(base_local_planner::Trajectory& traj)
{
    // Follow the global path
    // double dist;
    // double px, py, pth;
    // double closestDist = DBL_MAX;
    // double headingDiff = 0.0;
    // int interval = traj.getPointSize() / this->numPoints;
    // unsigned int closestIndex = 0;
    // for(unsigned int i = 0; i < this->numPoints; ++i){
    //     traj.getPoint((i + 1) * interval - 1, px, py, pth);
    //     for(unsigned int j = 1; j < this->global_plan.size(); ++j){
    //         dist = sqrt((this->global_plan[i].pose.position.x - px) * (this->global_plan[i].pose.position.x - px) + 
    //                     (this->global_plan[i].pose.position.y - py) * (this->global_plan[i].pose.position.y - py));
    //         if(dist < closestDist){
    //             closestDist = dist;
    //             closestIndex = j;
    //         }
    //     }
    //     couble v1_x,v1_y;
    //     v1_x = this->global_plan[closestIndex].pose.position.x - this->global_plan[closestIndex - 1].pose.position.x;
    //     v1_y = this->global_plan[closestIndex].pose.position.y - this->global_plan[closestIndex - 1].pose.position.y;
    //     double v2_x = cos(pth);
    //     double v2_y = sin(pth);

    //     double perp_dot = v1_x * v2_y - v2_x * v1_y;
    //     double dot = v1_x * v2_x + v1_y * v2_y;

    //     double diff = fabs(atan2(perp_dot, dot));
    //     if(diff > (M_PI / 2.0)){
    //         diff = fabs(diff - M_PI);
    //     }
    //     headingDiff = diff;
    // }
    // return headDiff;

    //Focus on the goal heading
    double dist = 0.0;
    double headingDiff = 0.0;
    int pointNum = traj.getPointsSize();
    double px,py,predTh;
    traj.getPoint(pointNum - 1, px, py, predTh);
    double goalTh = tf2::getYaw(goal.pose.orientation);
    predTh = angles::normalize_angle_positive(predTh);
    goalTh = angles::normalize_angle_positive(goalTh);
    headingDiff = fabs(predTh - goalTh);
    // std::cout << "predTh: " <<predTh  << "   goalTh : "<< goalTh << "    headingDiff: " << headingDiff << std::endl; 
    // return (-headingDiff) * this->scale;
    return (-headingDiff) * 0;
}

void AdwaHeadingCostFunction::setGlobalPlan(std::vector<geometry_msgs::PoseStamped> &plan)
{
    this->global_plan = plan;
    this->goal = global_plan.back();
}

void AdwaHeadingCostFunction::setScale(double scale)
{
    this->scale = scale;
}