#include "cross_point_cost.h"
#include <climits>
#include "angles/angles.h"


namespace base_local_planner{
    CrossPointCostFunction::CrossPointCostFunction()
    {

    }

    bool isVerticalLine(double angle, double eps)
    {
        if(abs(angle) >= 5 * M_PI / 12 + eps && abs(angle) <= 7 * M_PI / 12 - eps){
            return true;
        }
        return false;
    }
    std::pair<double,double> CrossPointCostFunction::ComputeCrossPoint(double x1,
                                                                       double y1,
                                                                       double th1,
                                                                       double x2,
                                                                       double y2,
                                                                       double th2)
    {
        double eps = 1e-4;
        double k1, b1, k2, b2;
        double x3, y3;

        k1 = std::tan(th1);
        k2 = std::tan(th2);
        b1 = y1 - k1 * x1;
        b2 = y2 - k2 * x2;
        if(isVerticalLine(k1, eps)){
            if(isVerticalLine(k2, eps)){
                return std::make_pair(INT_MAX, INT_MAX);
            } else {
                x3 = x1;
                y3 = k1 * x3 + b1;
            }
        } else {
            if(isVerticalLine(k2, eps)){
                x3 = x2;
                y3 = k2 * x3 + b2;
            } else {
                x3 = (b1 - b2) / (k2 - k1);
                y3 = k1 * x3 + b1;
            }
        }
        return std::make_pair(x3, y3);
    }

    double CrossPointCostFunction::scoreTrajectory(Trajectory &traj)
    {
        double xc, yc, thc;
        double xt, yt, tht;
        traj.getPoint(0, xc, yc, thc);
        xt = target_pose.pose.position.x;
        yt = target_pose.pose.position.y;
        tf2::Transform pose_tf;
        tf2::convert(target_pose.pose, pose_tf);
        double uselessP, uselessR;
        pose_tf.getBasis().getEulerYPR(tht, uselessP, uselessR);
        tht = angles::normalize_angle(tht);
        thc = angles::normalize_angle(thc);
        auto [px,py] = ComputeCrossPoint(xc, yc, thc, xt, yt, tht);
        int finalIndex = traj.getPointSize() - 1;
        double xf, yf, thf;
        traj.getPoint(finalIndex, xf, yf, thf);
        if(xf == INT_MAX && yf == INT_MAX){
            return 0;
        }
        double dist = std::sqrt(std::pow((xf - px), 2) + std::pow((yf - py), 2));
        return dist * crossPointScale_;
    }
}
