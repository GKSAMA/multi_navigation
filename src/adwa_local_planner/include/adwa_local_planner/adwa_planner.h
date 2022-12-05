#ifndef DWA_TEST_ADWA_PLANNER_H_
#define DWA_TEST_ADWA_PLANNER_H_

#include<vector>
#include<Eigen/Core>

#include<base_local_planner/map_grid_visualizer.h>
#include<base_local_planner/trajectory.h>
#include<base_local_planner/local_planner_limits.h>
#include<base_local_planner/local_planner_util.h>
#include<base_local_planner/simple_trajectory_generator.h>
#include<base_local_planner/oscillation_cost_function.h>
#include<base_local_planner/map_grid_cost_function.h>
#include<base_local_planner/obstacle_cost_function.h>
#include<base_local_planner/twirling_cost_function.h>
#include<base_local_planner/simple_scored_sampling_planner.h>
#include<costmap_2d/costmap_2d.h>
#include<nav_msgs/Path.h>

#include"adwa_local_planner/ADWAPlannerConfig.h"
#include"adwa_local_planner/desired_direction_cost.h"
#include"adwa_local_planner/vertical_distance_cost.h"
#include "adwa_local_planner/min_turn_radius_cost.h"
#include "adwa_local_planner/adwa_trajectory_sample_generator.h"
#include "adwa_local_planner/adwa_scored_sampling_planner.h"
#include "adwa_local_planner/ackermann_odometry_helper.h"
#include "adwa_local_planner/adwa_planner_utils.h"
#include "adwa_local_planner/heading_cost_function.h"
#include "adwa_local_planner/adwa_limits.h"

namespace adwa_local_planner{
    class ADWAPlanner{
    public:
        ADWAPlanner(std::string name,AdwaPlannerUtil* planner_util);

        ADWAPlanner();

        void reconfigure(adwa_local_planner::ADWAPlannerConfig &cfg);

        bool checkTrajectory(
            const Eigen::Vector3f pos,
            const Eigen::Vector3f vel,
            const Eigen::Vector3f vel_samples);
        // ackermann check trajectory
        bool checkTrajectory(
            const Eigen::Vector3f pos,
            const Eigen::Vector3f state,
            const Eigen::Vector2f vel_samples);

        base_local_planner::Trajectory findBestPath(
            const geometry_msgs::PoseStamped &global_pose,
            const geometry_msgs::PoseStamped &global_vel,
            geometry_msgs::PoseStamped& drive_velocities);
        
        // ackermann find best path
        base_local_planner::Trajectory findBestPath(
            const geometry_msgs::PoseStamped &global_pose,
            AckermannState state,
            geometry_msgs::PoseStamped& drive_velocities,
            std::vector<geometry_msgs::Point> footprint_spec);

        void updatePlanAndLocalCosts(
            const geometry_msgs::PoseStamped &global_pose,
            const std::vector<geometry_msgs::PoseStamped>& new_plan,
            const std::vector<geometry_msgs::Point>& footprint_spec,
            const geometry_msgs::PoseStamped& curVelocity);

        // ackermann update plan and local costs
        void updatePlanAndLocalCosts(
            const geometry_msgs::PoseStamped &global_pose,
            const std::vector<geometry_msgs::PoseStamped>& new_plan);

        double getSimPeriod() { return sim_period_; }

        bool getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost,float &total_cost);

        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

        bool setMaxAcc(double max_acc);
    private:
        AdwaPlannerUtil *planner_util_;

        double stop_time_buffer_;
        double path_distance_bias_, goal_distance_bias_, occdist_scale_, vertical_scale_, 
            direction_scale_, turning_scale_, heading_scale_;
        Eigen::Vector3f vsamples_;
        Eigen::Vector2f vsamples_ackermann_;    // ackermann velocity sample

        double sim_period_;
        base_local_planner::Trajectory result_traj_;

        double forward_point_distance_;// The distance from the center point of the robot to place an additional scoring point, in meters.
        std::vector<geometry_msgs::PoseStamped> global_plan_;

        boost::mutex configuration_mutex_;
        std::string frame_id_;
        ros::Publisher traj_cloud_pub_;
        bool publish_cost_grid_pc_; //whether or not to build and publish a PointCloud
        bool publish_traj_pc_;

        /**
         * @brief the maximum acceleration that user set.
         */
        double max_acc_init;
        double cheat_factor_;

        base_local_planner::MapGridVisualizer map_viz_;

        // base_local_planner::SimpleTrajectoryGenerator generator_;
        AdwaTrajectoryGenerator generator_;
        base_local_planner::OscillationCostFunction oscillation_costs_;
        base_local_planner::ObstacleCostFunction obstacle_costs_;
        base_local_planner::MapGridCostFunction path_costs_;
        base_local_planner::MapGridCostFunction goal_costs_;
        base_local_planner::MapGridCostFunction goal_front_costs_;
        base_local_planner::MapGridCostFunction alignment_costs_;
        base_local_planner::TwirlingCostFunction twirling_costs_;
        base_local_planner::DesiredDirectionCostFunction desired_direction_costs_;
        base_local_planner::VerticalDistanceCostFunction vertical_distance_costs_;
        base_local_planner::MinTurnRadiusCostFunction min_turn_radius_costs_;
        AdwaHeadingCostFunction heading_costs_;

        // base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;
        AdwaScoredSamplingPlanner scored_sampling_planner_;
    };
};
#endif