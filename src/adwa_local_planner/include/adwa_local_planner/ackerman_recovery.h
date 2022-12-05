#ifndef ACKERMAN_RECOVERY_H
#define ACKERMAN_RECOVERY_H

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/recovery_behavior.h>
#include <base_local_planner/costmap_model.h>
#include <tf2_ros/buffer.h>

namespace ackerman_recovery {
    class AckermanRecovery: public RecoveryBehavior {
    public:
        AckermanRecovery();
        void initialize();

        void runBehavior();
        ~AckermanRecovery();
    private:
        costmap_2d::Costmap2DROS* local_costmap_;
        bool initialized_;
        double sim_granularity_, frequency_, max_trans_vel_, min_trans_vel_;
        base_local_planner::CostmapModel* world_model_;
    };
};
#endif // ACKERMAN_RECOVERY_H