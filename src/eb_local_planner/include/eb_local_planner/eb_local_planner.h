//
// Created by gk on 2022/2/22.
//

#ifndef EB_LOCAL_PLANNER_EB_LOCAL_PLANNER_H
#define EB_LOCAL_PLANNER_EB_LOCAL_PLANNER_H
#include <ros/ros.h>
#include <ros/assert.h>

// classes which are part of this package
#include <eb_local_planner/conversions_and_types.h>
#include <eb_local_planner/eb_visualization.h>
#include <eb_local_planner/EBPlannerConfig.h>

// local planner specific classes
#include <base_local_planner/costmap_model.h>

// msgs
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>

// transforms
#include <angles/angles.h>

// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>

// boost classes
#include <boost/shared_ptr.hpp>

namespace eb_local_planner{
    class EBPlanner{
    public:
        EBPlanner();
        EBPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        ~EBPlanner();

        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        void reconfigure(EBPlannerConfig& config);
        void setVisualization(boost::shared_ptr<EBVisualization> eb_visual);
        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);
        bool getPlan(std::vector<geometry_msgs::PoseStamped>& global_plan);
        bool getBand(std::vector<Bubble>& elastic_band);
        bool addFrames(const std::vector<geometry_msgs::PoseStamped>& robot_pose, const AddAtPosition& add_frames_at);
        bool optimizeBand();
        bool optimizeBand(std::vector<Bubble>& band);

    private:
        costmap_2d::Costmap2DROS* costmap_ros_;
        std::vector<double> acc_lim_;
        int num_optim_iterations_;
        double internal_force_gain_;
        double external_force_gain_;
        double tiny_bubble_distance_;
        double tiny_bubble_expansion_;
        double min_bubble_overlap_;
        int max_recursion_depth_approx_equi_;
        double equilibrium_relative_overshoot_;
        double significant_force_;
        double costmap_weight_;
        base_local_planner::CostmapModel* world_model_;
        boost::shared_ptr<EBVisualization> eb_visual_;
        bool initialized_, visualization_;
        std::vector<geometry_msgs::Point> footprint_spec_;
        costmap_2d::Costmap2D* costmap_;
        std::vector<geometry_msgs::PoseStamped> global_plan_;
        std::vector<Bubble> elastic_band_;

        bool refineBand(std::vector<Bubble>& band);
        bool removeAndFill(std::vector<Bubble>& band, std::vector<Bubble>::iterator& start_iter,std::vector<Bubble>::iterator& end_iter);
        bool fillGap(std::vector<Bubble>& band, std::vector<Bubble>::iterator& start_iter,std::vector<Bubble>::iterator& end_iter);
        bool modifyBandArtificialForce(std::vector<Bubble>& band);
        bool applyForces(int bubble_num, std::vector<Bubble>& band, std::vector<geometry_msgs::WrenchStamped> forces);
        bool moveApproximateEquilibrium(const int& bubble_num,const std::vector<Bubble>& band, Bubble& curr_bubble,
                                        const geometry_msgs::WrenchStamped& curr_bubble_force, geometry_msgs::Twist& curr_step_width,
                                        const int& curr_recursion_depth);
        bool interpolateBubbles(geometry_msgs::PoseStamped start_center, geometry_msgs::PoseStamped end_center,geometry_msgs::PoseStamped& interpolated_center);
        bool checkOverlap(Bubble bubble1,Bubble bubble2);
        bool calcBubbleDistance(geometry_msgs::PoseStamped start_center, geometry_msgs::PoseStamped end_center,double& distance);
        bool calcBubbleDifference(geometry_msgs::PoseStamped start_center, geometry_msgs::PoseStamped end_center,geometry_msgs::Twist& difference);
        bool calcObstacleKinematicDistance(geometry_msgs::Pose center_pose, double& distance);
        bool getForcesAt(int bubble_num,std::vector<Bubble> band,Bubble curr_bubble,geometry_msgs::WrenchStamped& forces);
        bool calcInternalForces(int bubble_num,std::vector<Bubble> band, Bubble curr_bubble,geometry_msgs::WrenchStamped& forces);
        bool calcExternalForces(int bubble_num,Bubble curr_bubble,geometry_msgs::WrenchStamped forces);
        bool suppressTangentialForces(int bubble_num,std::vector<Bubble> band,geometry_msgs::WrenchStamped& forces);
        bool convertPlanToBand(std::vector<geometry_msgs::PoseStamped> plan,std::vector<Bubble>& band);
        bool convertBandToPlan(std::vector<geometry_msgs::PoseStamped>& plan,std::vector<Bubble> band);
    };
}

#endif //EB_LOCAL_PLANNER_EB_LOCAL_PLANNER_H
