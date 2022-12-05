#ifndef ADWA_SCORED_SAMPLING_PLANNER_H
#define ADWA_SCORED_SAMPLING_PLANNER_H

#include <vector>
#include <base_local_planner/trajectory.h>
#include <base_local_planner/trajectory_cost_function.h>
#include <base_local_planner/trajectory_sample_generator.h>
#include <base_local_planner/trajectory_search.h>

class AdwaScoredSamplingPlanner : public base_local_planner::TrajectorySearch
{
public:
    AdwaScoredSamplingPlanner() {}
    ~AdwaScoredSamplingPlanner() {}

    AdwaScoredSamplingPlanner(std::vector<base_local_planner::TrajectorySampleGenerator*> gen_list, 
                              std::vector<base_local_planner::TrajectoryCostFunction*>& critics, 
                              int max_samples = -1);

    double scoreTrajectory(base_local_planner::Trajectory& traj, double best_traj_cost);

    bool findBestTrajectory(base_local_planner::Trajectory& traj, std::vector<base_local_planner::Trajectory>* all_explored = 0);
private:
    std::vector<base_local_planner::TrajectorySampleGenerator*> gen_list_;
    std::vector<base_local_planner::TrajectoryCostFunction*> critics_;
    int max_samples_;
};

#endif // ADWA_SCORED_SAMPLING_PLANNER_H