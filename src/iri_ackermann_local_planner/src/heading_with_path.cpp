#include <sys/shm.h>
#include "heading_with_path.h"
#include "angles/angles.h"

namespace base_local_planner{
    HeadingWithPathCostFunction::HeadingWithPathCostFunction()
    {
        void *shared_memory = (void*)0;
        int shmid;
        shmid = shmget((key_t)0x9876, sizeof(CostState), 0666|IPC_CREAT);
        if(shmid == -1){
            std::cout << "shmget failed\n" << std::endl;
            exit(EXIT_FAILURE);
        }
        shared_memory = shmat(shmid, NULL, 0);
        costState_ = (CostState*)shared_memory;
    }

    double HeadingWithPathCostFunction::scoreTrajectory(Trajectory &traj)
    {
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
    }
}