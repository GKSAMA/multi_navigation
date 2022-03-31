//
// Created by gk on 2022/3/9.
//

#ifndef MULTI_TURTLEBOT3_NAVIGATION_UTILS_H
#define MULTI_TURTLEBOT3_NAVIGATION_UTILS_H

#include <ros/ros.h>
#include "tf/transform_listener.h"
#include "tf2/convert.h"
#include "tf2/transform_datatypes.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "visualization_msgs/Marker.h"
#include "angles/angles.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "vector"
#include "string"
#include "multi_turtlebot3_navigation/RunState.h"

namespace MultiNavUtil{

    struct runstate{
        bool isStart;
        bool isEnd;
        int waypoint;
        int idx;
    };

    /*!
    *  @brief set the runstate data
    *  @param isStart if the robot is running
    *  @param isEnd if the robot has stopped
    *  @param idx which robot
    */
    void setRunstate(runstate *rs,bool isStart,bool isEnd, int waypoint, int idx){
        rs->idx = idx;
        rs->isStart = isStart;
        rs->isEnd = isEnd;
        rs->waypoint = waypoint;
    }
    /*!
    * @brief pose to pose2D
    * @param pose
    * @param pose2D 
    */
    void PoseToPose2D(const geometry_msgs::Pose pose, geometry_msgs::Pose2D &pose2D);
    /*!
    * @brief pose2D to pose
    * @param pose2D
    * @param pose 
    */
    void Pose2DToPose(const geometry_msgs::Pose2D pose2D, geometry_msgs::Pose &pose);
    /*!
     * @brief convert transform data to MoveBaseGoal data
     * @param goal
     * @param transform
     */
    void setPose(move_base_msgs::MoveBaseGoal &goal,tf::StampedTransform transform);
    /*!
     * @brief convert transform data to pose data
     * @param pose
     * @param transform
     */
    void setPose(geometry_msgs::Pose &pose,tf::StampedTransform transform);
    /*!
     * @brief convert transform data to point data
     * @param point
     * @param transform
     */
    void setPoint(geometry_msgs::Point &point,tf::StampedTransform transform);
    /*!
     * @brief To stop the robot by sending a non-zero twist command firstly then sending a zero twist command
     * @param pub
     */
    void stopRobot(const ros::Publisher& pub);
    /*!
     * @brief Judge if the robot has stopped by compare the current pose and the last pose
     * @param curr_transform
     * @param last_pose
     * @param pos_stop_threshold the minimum distance the robot can travel
     * @param ang_stop_threshold the minimum angle the robot can rotate
     * @param index which robot
     * @return
     */
    bool isRobotStop(const tf::StampedTransform &curr_transform, const geometry_msgs::Pose &last_pose,
                     const double &pos_stop_threshold,const double &ang_stop_threshold,const int &index);
    /*!
     * @brief calculate the run time of the robot between two stop states
     * @param curr_transform
     * @param last_pose
     * @param time total time
     * @param start_time the time when the robot start travel
     * @param end_time the time when the robot stop
     * @param pos_stop_threshold the minimum distance the robot can travel
     * @param ang_stop_threshold the minimum angle the robot can rotate
     * @param index which robot
     */
    void calcTotalTime(const tf::StampedTransform &curr_transform, const geometry_msgs::Pose &last_pose,
                       double &time,ros::Time &start_time,ros::Time &end_time,
                       const double pos_stop_threshold,const double ang_stop_threshold,const int &index);
    /*!
     * @brief calculate the run time of the robot between two waypoints
     * @param curr_transform
     * @param last_pose
     * @param time total time
     * @param start_time the time when the robot start travel
     * @param end_time the time when the robot stop
     * @param runstate the run state of the robot
     */
    void calcTotalTime(const tf::StampedTransform &curr_transform, const geometry_msgs::Pose &last_pose,
                       double &time,ros::Time &start_time,ros::Time &end_time, 
                       const MultiNavUtil::runstate *runstate, const int index);
    /*!
     * @brief calculate the travel distance of the robot between two stop states
     * @param curr_transform
     * @param last_pose
     * @param pos_stop_threshold the minimum distance the robot can travel
     * @param ang_stop_threshold the minimum angle the robot can rotate
     * @param total_distance total distance
     * @param index which robot
     */
    void calcTotalDistance(const tf::StampedTransform &curr_transform, const geometry_msgs::Pose& last_pose,
                           const double pos_stop_threshold,const double ang_stop_threshold, double &total_distance,
                           const int index);

    /*!
     * @brief calculate the travel distance of the robot between two waypoints
     * @param curr_transform
     * @param last_pose
     * @param total_distance
     * @param runstate the run state of the robot
     */
    void calcTotalDistance(const tf::StampedTransform &curr_transform, const geometry_msgs::Pose& last_pose,
                           double &total_distance, const MultiNavUtil::runstate* runstate, const int index);

    bool is_start0(false),is_start1(false),is_start2(false);
    bool is_end0(false),is_end1(false),is_end2(false);
}

#endif //MULTI_TURTLEBOT3_NAVIGATION_UTILS_H
