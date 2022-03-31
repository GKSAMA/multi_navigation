#ifndef DWA_TEST_ADWA_PLANNER_ROS_H_
#define DWA_TEST_ADWA_PLANNER_ROS_H_

#include<boost/shared_ptr.hpp>
#include<boost/thread.hpp>

#include<tf/transform_listener.h>
#include<dwa_local_planner/DWAPlannerConfig.h>

#include<angles/angles.h>

#include<nav_msgs/Odometry.h>

#include<costmap_2d/costmap_2d_ros.h>
#include<nav_core/base_local_planner.h>
#include<base_local_planner/latched_stop_rotate_controller.h>

#include<base_local_planner/odometry_helper_ros.h>

#include<adwa_planner.h>


namespace adwa_local_planner{
    class ADWAPlannerROS:public nav_core::BaseLocalPlanner{
        public:
            ADWAPlannerROS();

            void initialize(std::string name,tf2_ros::Buffer* tf,
            costmap_2d::Costmap2DROS* costmap_ros);

            ~ADWAPlannerROS();

            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

            bool adwaComputeVelocityCommands(geometry_msgs::PoseStamped& global_pose,geometry_msgs::Twist& cmd_vel);

            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

            bool isGoalReached();

            bool isInitialized(){
                return initialized_;
            }

        private:
            void reconfigureCB(dwa_local_planner::DWAPlannerConfig &config, uint32_t level);

            void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);
            
            void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

            tf2_ros::Buffer* tf_;

            ros::Publisher g_plan_pub_, l_plan_pub_;

            base_local_planner::LocalPlannerUtil planner_util_;

            boost::shared_ptr<ADWAPlanner> dp_;

            costmap_2d::Costmap2DROS* costmap_ros_;

            dynamic_reconfigure::Server<dwa_local_planner::DWAPlannerConfig> *dsrv_;
            dwa_local_planner::DWAPlannerConfig default_config_;
            bool setup_;
            geometry_msgs::PoseStamped current_pose_;

            base_local_planner::LatchedStopRotateController latchedStopRotateController_;

            bool initialized_;

            base_local_planner::OdometryHelperRos odom_helper_;
            std::string odom_topic_;

            double max_acc_theta;
    };
};
#endif