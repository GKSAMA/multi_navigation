#ifndef ADWA_TRAJECTORY_SAMPLE_GENERATOR_H
#define ADWA_TRAJECTORY_SAMPLE_GENERATOR_H

#include <base_local_planner/trajectory_sample_generator.h>
#include <adwa_local_planner/adwa_limits.h>
#include <Eigen/Core>

class AdwaTrajectoryGenerator : public base_local_planner::TrajectorySampleGenerator
{
public:
    AdwaTrajectoryGenerator();

    void initialise(const Eigen::Vector3f& pos, 
                    const Eigen::Vector3f& state, 
                    const Eigen::Vector3f& goal,
                    AdwaLimits* limits, 
                    const Eigen::Vector2f& vsamples,
                    bool discretize_bt_time = false);

    void setParameters(double max_sim_time, 
                        double min_sim_time,
                        double sim_granularity,
                        double angular_sim_granularity,
                        double sim_period = 0.0);
    
    bool hasMoreTrajectories(void);

    bool nextTrajectory(base_local_planner::Trajectory &traj);

    bool generateTrajectory(Eigen::Vector3f pos, 
                            Eigen::Vector3f vel, 
                            Eigen::Vector2f sample_target_vel,
                            base_local_planner::Trajectory& traj);
    
    Eigen::Vector3f computeNewPosition(const Eigen::Vector3f& pos, 
                                    const Eigen::Vector2f& vel,
                                    double dt);

    Eigen::Vector2f computeNewVelocity(double trans_vel, 
                                        double steer_angle, 
                                        double dt, 
                                        Eigen::Vector2f sample_target_vel,
                                        double &steer_vel);

    ~AdwaTrajectoryGenerator();

protected:
    unsigned int next_sample_index_;
    std::vector<Eigen::Vector2f> sample_params_;
    AdwaLimits* limits_;
    Eigen::Vector3f pos_;
    Eigen::Vector3f state_;

    bool discretize_bt_time_;
    double max_sim_time_, min_sim_time_, sim_time_;
    double sim_granularity_, angular_sim_granularity_;
    double sim_period_;
};

#endif // ADWA_TRAJECTORY_SAMPLE_GENERATOR_H