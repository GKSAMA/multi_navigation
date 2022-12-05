#ifndef ADWA_PLANNER_UTILS
#define ADWA_PLANNER_UTILS

#include <thread>
#include <mutex>

#include <costmap_2d/costmap_2d.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <nav_core/base_local_planner.h>

#include "adwa_local_planner/adwa_limits.h"

class AdwaPlannerUtil
{
private:
    std::string name_;
    std::string global_frame_;
    costmap_2d::Costmap2D* costmap_;
    tf2_ros::Buffer* tf_;

    std::vector<geometry_msgs::PoseStamped> global_plan_;

    std::mutex limits_configuration_mutex_;
    bool setup_;
    AdwaLimits default_limits_;
    AdwaLimits limits_;
    bool initialized_;

    std::vector < std::vector<geometry_msgs::PoseStamped> > subPathList;
    std::vector < geometry_msgs::PoseStamped > subPath;
    unsigned int subPathIndex;
public:
    AdwaPlannerUtil();
    ~AdwaPlannerUtil();

    void reconfigureCB(AdwaLimits &config, bool restore_defaults);

    void initialize(tf2_ros::Buffer* tf, costmap_2d::Costmap2D* costmap, std::string global_frame);

    bool getGoal(geometry_msgs::PoseStamped& global_pose);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);

    bool setNextPath();

    bool lastPath();

    bool getLocalPlan(geometry_msgs::PoseStamped &global_pose, std::vector<geometry_msgs::PoseStamped> &transformed_plan);

    costmap_2d::Costmap2D* getCostmap();

    AdwaLimits getCurrentLimits();

    std::string getGlobalFrame();
};

#endif // ADWA_PLANNER_UTILS