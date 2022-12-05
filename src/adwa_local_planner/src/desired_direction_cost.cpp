#include "adwa_local_planner/desired_direction_cost.h"
#include "angles/angles.h"

namespace base_local_planner{

    bool DesiredDirectionCostFunction::prepare(){
        return true;
    }

    void DesiredDirectionCostFunction::SetTargetPose(geometry_msgs::PoseStamped target){
        target_pose = target;
    }

    void DesiredDirectionCostFunction::SetScale(double scale) {
        direction_scale = scale;
    }

    double DesiredDirectionCostFunction::scoreTrajectory(Trajectory &traj){
        double tx, ty, tth;
        double px, py, pth, endth;
        int finalIndex = traj.getPointsSize() - 1;
        traj.getPoint(finalIndex, px, py, pth);
        endth = angles::normalize_angle(pth);
        tf2::Transform pose_tf;
        tf2::convert(target_pose.pose,pose_tf);
        double useless_pitch, useless_roll;
        pose_tf.getBasis().getEulerYPR(tth, useless_pitch, useless_roll);
        tth = angles::normalize_angle(tth);
        double thMax = 0;
        for(unsigned int i = 0; i < traj.getPointsSize(); ++i){
            traj.getPoint(i, px, py, pth);
            pth = angles::normalize_angle(pth);
            thMax = std::max(thMax, std::abs(pth - tth));
        }
        return (1 - std::abs(endth - tth) / thMax) * direction_scale;
    }
};
