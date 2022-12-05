#ifndef DWA_TEST_ADWA_PLANNER_ROS_H_
#define DWA_TEST_ADWA_PLANNER_ROS_H_

#include<boost/shared_ptr.hpp>
#include<boost/thread.hpp>

#include<tf/transform_listener.h>
#include<angles/angles.h>
#include<nav_msgs/Odometry.h>
#include<costmap_2d/costmap_2d_ros.h>
#include<nav_core/base_local_planner.h>
#include<base_local_planner/latched_stop_rotate_controller.h>
#include<base_local_planner/odometry_helper_ros.h>

#include "adwa_local_planner/adwa_planner.h"
#include "adwa_local_planner/ADWAPlannerConfig.h"
#include "adwa_local_planner/adwa_trajectory_sample_generator.h"
#include "adwa_local_planner/ackermann_odometry_helper.h"
#include "adwa_local_planner/adwa_limits.h"
#include "adwa_local_planner/adwa_planner_utils.h"
#include "adwa_local_planner/adwa_scored_sampling_planner.h"


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

        geometry_msgs::Twist averageCmdVel(geometry_msgs::Twist &new_cmd_vel);

    private:
        void reconfigureCB(adwa_local_planner::ADWAPlannerConfig &config, uint32_t level);

        void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);
        
        void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

        tf2_ros::Buffer* tf_;

        ros::Publisher g_plan_pub_, l_plan_pub_;

        AdwaPlannerUtil planner_util_;

        boost::shared_ptr<ADWAPlanner> dp_;

        costmap_2d::Costmap2DROS* costmap_ros_;

        dynamic_reconfigure::Server<adwa_local_planner::ADWAPlannerConfig> *dsrv_;
        adwa_local_planner::ADWAPlannerConfig default_config_;
        bool setup_;
        geometry_msgs::PoseStamped current_pose_;

        base_local_planner::LatchedStopRotateController latchedStopRotateController_;

        bool initialized_;

        // base_local_planner::OdometryHelperRos odom_helper_;
        AckermannOdometry odom_helper_;
        std::string odom_topic_;

        double max_acc_theta;
        // ackermann config
        std::vector<geometry_msgs::Twist> last_cmds_;
        bool stucked_;
        bool first_;
        int patience_;
    };
};
#endif