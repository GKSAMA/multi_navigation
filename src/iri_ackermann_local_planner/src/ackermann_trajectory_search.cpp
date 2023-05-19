/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: TKruse
 *********************************************************************/

#include <ackermann_trajectory_search.h>
#include "vertical_distance_cost.h"
#include "desired_direction_cost.h"

#include <sys/shm.h>
#include <ros/console.h>

AckermannTrajectorySearch::AckermannTrajectorySearch(std::vector<base_local_planner::TrajectorySampleGenerator *> gen_list, std::vector<base_local_planner::TrajectoryCostFunction *> &critics, int max_samples)
{
  max_samples_ = max_samples;
  gen_list_ = gen_list;
  critics_ = critics;
}

double AckermannTrajectorySearch::scoreTrajectory(base_local_planner::Trajectory &traj, double best_traj_cost)
{
  double traj_cost = 0;
  int gen_id = 0;
  for (std::vector<base_local_planner::TrajectoryCostFunction *>::iterator score_function = critics_.begin(); score_function != critics_.end(); ++score_function)
  {
    base_local_planner::TrajectoryCostFunction *score_function_p = *score_function;
    if (score_function_p->getScale() == 0)
      continue;
    double cost = score_function_p->scoreTrajectory(traj);
    // std::cout << "gen_id" << gen_id << ": ";
    // std::cout << "cost=" << cost << " " << "scale=" << score_function_p->getScale() <<" ";
    if (cost < 0)
    {
      ROS_DEBUG("Velocity %.3lf, %.3lf, %.3lf discarded by cost function  %d with cost: %f", traj.xv_, traj.yv_, traj.thetav_, gen_id, cost);
      traj_cost = cost;
      break;
    }
    if (cost != 0)
      cost *= score_function_p->getScale();
    // std::cout << "score=" << cost << " ";
    traj_cost += cost;
    if (best_traj_cost > 0)
    {
      // since we keep adding positives, once we are worse than the best, we will stay worse
      if (traj_cost > best_traj_cost)
        break;
    }
    gen_id++;
    ROS_DEBUG("Velocity %.3lf, %.3lf, %.3lf evaluated by cost function %d with cost: %f (best cost: %f)", traj.xv_, traj.yv_, traj.thetav_, gen_id, cost, best_traj_cost);
  }
  // std::cout << "traj_cost = " << traj_cost<<std::endl;
  return traj_cost;
}

bool AckermannTrajectorySearch::findBestTrajectory(base_local_planner::Trajectory &traj, std::vector<base_local_planner::Trajectory> *all_explored)
{
  base_local_planner::Trajectory loop_traj;
  base_local_planner::Trajectory best_traj;
  double loop_traj_cost, best_traj_cost = -1;
  bool gen_success;
  int count, count_valid, num = 0;
  for (std::vector<base_local_planner::TrajectoryCostFunction *>::iterator loop_critic = critics_.begin(); loop_critic != critics_.end(); ++loop_critic)
  {
    base_local_planner::TrajectoryCostFunction *loop_critic_p = *loop_critic;
    if (loop_critic_p->prepare() == false)
    {
      ROS_WARN("A scoring function failed to prepare");
      return false;
    }
  }

  for (std::vector<base_local_planner::TrajectorySampleGenerator *>::iterator loop_gen = gen_list_.begin(); loop_gen != gen_list_.end(); ++loop_gen)
  {
    count = 0;
    count_valid = 0;
    base_local_planner::TrajectorySampleGenerator *gen_ = *loop_gen;
    while (gen_->hasMoreTrajectories())
    {
      gen_success = gen_->nextTrajectory(loop_traj);
      if (gen_success == false)
      {
        // TODO use this for debugging
        continue;
      }
      loop_traj_cost = scoreTrajectory(loop_traj, best_traj_cost);
      
      //////////////////// shared memory log test block //////////////////////////////
      // void *shared_memory = (void*)0;
      // CostState *cs;
      // int shmid = shmget((key_t)0x9876, sizeof(CostState), 0666 | IPC_CREAT);
      // if(shmid == -1){
      //     std::cout << "Direction shmget failed in search..." << std::endl;
      // }
      // shared_memory = shmat(shmid, NULL, 0);
      // // perror("shmat");
      // if(shared_memory == (void*)-1){
      //     std::cout << "Direction shmat failed in search..." << std::endl;
      // }
      // cs = (CostState*)shared_memory;
      // std::cout << "traj end x = " << cs->Traj_x << "  traj end y = " << cs->Traj_y << std::endl;
      // std::cout << "trajectory cost = " << loop_traj_cost << "  dir_diff = " << cs->DirDiff << "  dir_cost = " << cs->DirCost << std::endl;
      // std::cout << "vertical distance = " << cs->VertDist << "  vertical cost = " << cs->VertCost << std::endl;
      //////////////////// shared memory log test block //////////////////////////////

      if (all_explored != NULL)
      {
        loop_traj.cost_ = loop_traj_cost;
        all_explored->push_back(loop_traj);
      }
      if (loop_traj_cost >= 0)
      {
        count_valid++;
        if (best_traj_cost < 0 || loop_traj_cost < best_traj_cost)
        {
          best_traj_cost = loop_traj_cost;
          best_traj = loop_traj;
          //////////////////// shared memory log test block //////////////////////////////
          // dirDiff = cs->DirDiff;
          // dirCost = cs->DirCost;
          // vertDist = cs->VertDist;
          // vertCost = cs->VertCost;
          // trajEndX = cs->Traj_x;
          // trajEndY = cs->Traj_y;
          // std::cout << "traj end x = " << cs->Traj_x << "  traj end y = " << cs->Traj_y << std::endl;
          // std::cout << "trajectory cost = " << loop_traj_cost << "  dir_diff = " << cs->DirDiff << "  dir_cost = " << cs->DirCost << std::endl;
          // std::cout << "vertical distance = " << cs->VertDist << "  vertical cost = " << cs->VertCost << std::endl;
          //////////////////// shared memory log test block //////////////////////////////
        }
      }
      count++;
      if (max_samples_ > 0 && count >= max_samples_)
        break;
    }
    if (best_traj_cost >= 0)
    {
      traj.xv_ = best_traj.xv_;
      traj.yv_ = best_traj.yv_;
      traj.thetav_ = best_traj.thetav_;
      traj.cost_ = best_traj_cost;
      traj.resetPoints();
      double px, py, pth;
      for (unsigned int i = 0; i < best_traj.getPointsSize(); i++)
      {
        best_traj.getPoint(i, px, py, pth);
        traj.addPoint(px, py, pth);
      }
    }
    ROS_DEBUG("Evaluated %d trajectories, found %d valid", count, count_valid);
    if (best_traj_cost >= 0)
    {
      // do not try fallback generators
      break;
    }
  }
  return best_traj_cost >= 0;
}
