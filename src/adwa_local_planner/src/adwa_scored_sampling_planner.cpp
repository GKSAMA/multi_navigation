#include <adwa_local_planner/adwa_scored_sampling_planner.h>
#include <ros/console.h>

AdwaScoredSamplingPlanner::AdwaScoredSamplingPlanner(std::vector<base_local_planner::TrajectorySampleGenerator*> gen_list, 
                              std::vector<base_local_planner::TrajectoryCostFunction*>& critics, 
                              int max_samples)
{
    gen_list_ = gen_list;
    critics_ = critics;
    max_samples_ = max_samples;
}                              

double AdwaScoredSamplingPlanner::scoreTrajectory(base_local_planner::Trajectory& traj, double best_traj_cost)
{
    double traj_cost = 0;
    int gen_id = 0;
    for(std::vector<base_local_planner::TrajectoryCostFunction*>::iterator score_function = critics_.begin(); score_function != critics_.end(); ++score_function){
        base_local_planner::TrajectoryCostFunction* score_function_p = *score_function;
        if(score_function_p->getScale() == 0){
            continue;
        }
        double cost = score_function_p->scoreTrajectory(traj);
        if(cost < 0){
            ROS_DEBUG("Velocity %.3lf, %.3lf, %.3lf discarded by cost function  %d with cost: %f", traj.xv_, traj.yv_, traj.thetav_, gen_id, cost);
            traj_cost = cost;
            break;
        }
        if(cost != 0){
            cost *= score_function_p->getScale();
        }
        traj_cost += cost;
        if(best_traj_cost > 0){
            if(traj_cost > best_traj_cost){
                break;
            }
            ++gen_id;
        }
        ROS_DEBUG("Velocity %.3lf, %.3lf, %.3lf evaluated by cost function %d with cost: %f (best cost: %f)", traj.xv_, traj.yv_, traj.thetav_, gen_id, cost, best_traj_cost);
    }
    return traj_cost;
}

bool AdwaScoredSamplingPlanner::findBestTrajectory(base_local_planner::Trajectory& traj, std::vector<base_local_planner::Trajectory>* all_explored)
{
    base_local_planner::Trajectory loop_traj;
    base_local_planner::Trajectory best_traj;
    double loop_traj_cost, best_traj_cost = -1;
    bool gen_success;
    int count, count_valid, num = 0;
    for(std::vector<base_local_planner::TrajectoryCostFunction*>::iterator loop_critic = critics_.begin(); loop_critic != critics_.end(); ++loop_critic){
        base_local_planner::TrajectoryCostFunction* loop_critic_p = *loop_critic;
        if(!loop_critic_p->prepare()){
            ROS_WARN("A scoring function failed to prepare");
            return false;
        }
    }
    for(std::vector<base_local_planner::TrajectorySampleGenerator*>::iterator loop_gen = gen_list_.begin(); loop_gen != gen_list_.end(); ++loop_gen){
        count = 0;
        count_valid = 0;
        base_local_planner::TrajectorySampleGenerator* gen_ = *loop_gen;
        while(gen_->hasMoreTrajectories()){
            gen_success = gen_->nextTrajectory(loop_traj);
            if(!gen_success){
                continue;
            }
            loop_traj_cost = scoreTrajectory(loop_traj, best_traj_cost);
            if(all_explored != NULL){
                loop_traj.cost_ = loop_traj_cost;
                all_explored->emplace_back(loop_traj);
            }
            if(loop_traj_cost >= 0){
                ++count_valid;
                if(best_traj_cost < 0 || loop_traj_cost < best_traj_cost){
                    best_traj_cost = loop_traj_cost;
                    best_traj = loop_traj;
                }
            }
            ++count;
            if(max_samples_ > 0 && count >= max_samples_){
                break;
            }
            // ROS_INFO("trajectory cost: %.3f, velocityX: %.3f, velocityY: %.3f, velocityTh: %.3f", loop_traj.cost_, loop_traj.xv_, loop_traj.yv_,loop_traj.thetav_);
        }
        if(best_traj_cost >= 0){
            traj.xv_ = best_traj.xv_;
            traj.yv_ = best_traj.yv_;
            traj.thetav_ = best_traj.thetav_;
            traj.cost_ = best_traj_cost;
            traj.resetPoints();
            double px,py,pth;
            for(unsigned int i = 0; i < best_traj.getPointsSize(); ++i){
                best_traj.getPoint(i, px, py, pth);
                traj.addPoint(px, py, pth);
            }
        }
        // ROS_INFO("best trajectory cost: %.3f, velocityX: %.3f, velocityY: %.3f, velocityTh: %.3f", best_traj.cost_, best_traj.xv_, best_traj.yv_,best_traj.thetav_);
        ROS_DEBUG("Evaluated %d trajectories, found %d valid", count, count_valid);
        if(best_traj_cost >= 0){
            break;
        }
    }
    return best_traj_cost >= 0;
}