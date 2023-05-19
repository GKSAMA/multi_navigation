/*
 * @Author: Gong, Ke gk.kingj@gmail.com
 * @Date: 2022-10-21 12:17:34
 * @LastEditors: Gong, Ke gk.kingj@gmail.com
 * @LastEditTime: 2023-04-22 06:22:03
 * @FilePath: /multi_turtlebot3_navigation/src/iri_ackermann_local_planner/src/desired_direction_cost.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <sys/shm.h>
#include "desired_direction_cost.h"
#include "angles/angles.h"

namespace base_local_planner{
    DesiredDirectionCostFunction::DesiredDirectionCostFunction()
    {
        void *shared_memory = (void*)0;
        int shmid;
        // shmid = shmget((key_t)4321, 0, 0);
        // if(shmid != -1){
        //     shmctl(shmid, IPC_RMID, 0);
        // }
        shmid = shmget((key_t)0x9876, sizeof(CostState), 0666|IPC_CREAT);
        if(shmid == -1){
            std::cout << "shmget failed\n" << std::endl;
            exit(EXIT_FAILURE);
        }
        shared_memory = shmat(shmid, NULL, 0);
        costState_ = (CostState*)shared_memory;
    }

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
        // double thMax = 0;
        // for(unsigned int i = 0; i < traj.getPointsSize(); ++i){
        //     traj.getPoint(i, px, py, pth);
        //     pth = angles::normalize_angle(pth);
        //     thMax = std::max(thMax, std::abs(pth - tth));
        // }
        costState_->DirDiff = std::abs(endth - tth);
        if(costState_->DirDiff > M_PI){
            costState_->DirDiff = 2 * M_PI - costState_->DirDiff;
        }
        // costState_->DirCost = (1 - std::abs(endth - tth) / thMax) * direction_scale;
        costState_->DirCost = std::abs(endth - tth);
        // return (1 - std::abs(endth - tth) / thMax) * direction_scale;
        return costState_->DirCost;
    }
};
