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

#ifndef ACKERMANN_TRAJECTORY_SEARCH_H_
#define ACKERMANN_TRAJECTORY_SEARCH_H_

#include <vector>
#include <base_local_planner/trajectory.h>
#include <base_local_planner/trajectory_cost_function.h>
#include <base_local_planner/trajectory_sample_generator.h>
#include <base_local_planner/trajectory_search.h>


/**
 * @class AckermannTrajectorySearch
 * @brief Generates a local plan using the given generator and cost functions.
 * Assumes less cost are best, and negative costs indicate infinite costs
 *
 * This is supposed to be a simple and robust implementation of
 * the TrajectorySearch interface. More efficient search may well be
 * possible using search heuristics, parallel search, etc.
 */
class AckermannTrajectorySearch : public base_local_planner::TrajectorySearch {
public:

  ~AckermannTrajectorySearch() {}

  AckermannTrajectorySearch() {}

  /**
   * Takes a list of generators and critics. Critics return costs > 0, or negative costs for invalid trajectories.
   * Generators other than the first are fallback generators,  meaning they only get to generate if the previous
   * generator did not find a valid trajectory.
   * Will use every generator until it stops returning trajectories or count reaches max_samples.
   * Then resets count and tries for the next in the list.
   * passing max_samples = -1 (default): Each Sampling planner will continue to call
   * generator until generator runs out of samples (or forever if that never happens)
   */
  AckermannTrajectorySearch(std::vector<base_local_planner::TrajectorySampleGenerator*> gen_list, std::vector<base_local_planner::TrajectoryCostFunction*>& critics, int max_samples = -1);

  /**
   * runs all scoring functions over the trajectory creating a weigthed sum
   * of positive costs, aborting as soon as a negative cost are found or costs greater
   * than positive best_traj_cost accumulated
   */
  double scoreTrajectory(base_local_planner::Trajectory& traj, double best_traj_cost);

  /**
   * Calls generator until generator has no more samples or max_samples is reached.
   * For each generated traj, calls critics in turn. If any critic returns negative
   * value, that value is assumed as costs, else the costs are the sum of all critics
   * result. Returns true and sets the traj parameter to the first trajectory with
   * minimal non-negative costs if sampling yields trajectories with non-negative costs,
   * else returns false.
   *
   * @param traj The container to write the result to
   * @param all_explored pass NULL or a container to collect all trajectories for debugging (has a penalty)
   */
  bool findBestTrajectory(base_local_planner::Trajectory& traj, std::vector<base_local_planner::Trajectory>* all_explored = 0);

  double dirDiff;
  double dirCost;
  double vertDist;
  double vertCost;
  double trajEndX;
  double trajEndY;
private:
  std::vector<base_local_planner::TrajectorySampleGenerator*> gen_list_;
  std::vector<base_local_planner::TrajectoryCostFunction*> critics_;

  int max_samples_;
};

#endif /* SIMPLE_SCORED_SAMPLING_PLANNER_H_ */
