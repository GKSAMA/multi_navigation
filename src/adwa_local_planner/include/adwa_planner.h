#ifndef DWA_TEST_ADWA_PLANNER_H_
#define DWA_TEST_ADWA_PLANNER_H_

#include<vector>
#include<Eigen/Core>

#include<dwa_local_planner/DWAPlannerConfig.h>

#include<base_local_planner/map_grid_visualizer.h>

#include<costmap_2d/costmap_2d.h>

#include<base_local_planner/trajectory.h>
#include<base_local_planner/local_planner_limits.h>
#include<base_local_planner/local_planner_util.h>
#include<base_local_planner/simple_trajectory_generator.h>
#include<base_local_planner/oscillation_cost_function.h>
#include<base_local_planner/map_grid_cost_function.h>
#include<base_local_planner/obstacle_cost_function.h>
#include<base_local_planner/twirling_cost_function.h>
#include<base_local_planner/simple_scored_sampling_planner.h>

#include<nav_msgs/Path.h>

namespace adwa_local_planner{
    class ADWAPlanner{
        public:
            ADWAPlanner(std::string name,base_local_planner::LocalPlannerUtil* planner_util);

            ADWAPlanner();

            void reconfigure(dwa_local_planner::DWAPlannerConfig &cfg);

            bool checkTrajectory(
                const Eigen::Vector3f pos,
                const Eigen::Vector3f vel,
                const Eigen::Vector3f vel_samples);

            base_local_planner::Trajectory findBestPath(
                const geometry_msgs::PoseStamped &global_pose,
                const geometry_msgs::PoseStamped &global_vel,
                geometry_msgs::PoseStamped& drive_velocities);

            void updatePlanAndLocalCosts(
                const geometry_msgs::PoseStamped &global_pose,
                const std::vector<geometry_msgs::PoseStamped>& new_plan,
                const std::vector<geometry_msgs::Point>& footprint_spec);

            double getSimPeriod() { return sim_period_; }

            bool getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost,float &total_cost);

            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

        private:
            base_local_planner::LocalPlannerUtil *planner_util_;

            double stop_time_buffer_;
            double path_distance_bias_, goal_distance_bias_, occdist_scale_;
            Eigen::Vector3f vsamples_;

            double sim_period_;
            base_local_planner::Trajectory result_traj_;

            double forward_point_distance_;// The distance from the center point of the robot to place an additional scoring point, in meters.
            std::vector<geometry_msgs::PoseStamped> global_plan_;

            boost::mutex configuration_mutex_;
            std::string frame_id_;
            ros::Publisher traj_cloud_pub_;
            bool publish_cost_grid_pc_; //whether or not to build and publish a PointCloud
            bool publish_traj_pc_;

            double cheat_factor_;

            base_local_planner::MapGridVisualizer map_viz_;

            base_local_planner::SimpleTrajectoryGenerator generator_;
            base_local_planner::OscillationCostFunction oscillation_costs_;
            base_local_planner::ObstacleCostFunction obstacle_costs_;
            base_local_planner::MapGridCostFunction path_costs_;
            base_local_planner::MapGridCostFunction goal_costs_;
            base_local_planner::MapGridCostFunction goal_front_costs_;
            base_local_planner::MapGridCostFunction alignment_costs_;
            base_local_planner::TwirlingCostFunction twirling_costs_;

            base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;
    };
};
#endif