//
// Created by gk on 2022/3/9.
//

#include "multi_turtlebot3_navigation/utils.h"

namespace MultiNavUtil{
    
    double tiny_offset = 0.005;
    double tiny_time = 0.5;

    void PoseToPose2D(const geometry_msgs::Pose pose, geometry_msgs::Pose2D &pose2D){
        tf2::Transform pose_tf;
        tf2::convert(pose,pose_tf);
        double useless_pitch, useless_roll, yaw;
        pose_tf.getBasis().getEulerYPR(yaw,useless_pitch,useless_roll);

        yaw = angles::normalize_angle(yaw);
        pose2D.x = pose.position.x;
        pose2D.y = pose.position.y;
        pose2D.theta = yaw;
    }

    void Pose2DToPose(const geometry_msgs::Pose2D pose2D, geometry_msgs::Pose &pose){
        tf2::Quaternion frame_quat;
        frame_quat.setRPY(0,0,pose2D.theta);
        pose.position.x = pose2D.x;
        pose.position.y = pose2D.y;
        pose.position.z = 0;
        pose.orientation.x = frame_quat.x();
        pose.orientation.y = frame_quat.y();
        pose.orientation.z = frame_quat.z();
        pose.orientation.w = frame_quat.w();
    }

    void setPose(move_base_msgs::MoveBaseGoal &goal,const tf::StampedTransform transform){
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = transform.getOrigin().x();
        goal.target_pose.pose.position.y = transform.getOrigin().y();
        goal.target_pose.pose.position.z = transform.getOrigin().z();
        goal.target_pose.pose.orientation.w = transform.getRotation().w();
        goal.target_pose.pose.orientation.x = transform.getRotation().x();
        goal.target_pose.pose.orientation.y = transform.getRotation().y();
        goal.target_pose.pose.orientation.z = transform.getRotation().z();
    }

    void setPose(geometry_msgs::Pose &pose, const tf::StampedTransform transform){
        pose.position.x = transform.getOrigin().x();
        pose.position.y = transform.getOrigin().y();
        pose.position.z = transform.getOrigin().z();
        pose.orientation.w = transform.getRotation().w();
        pose.orientation.x = transform.getRotation().x();
        pose.orientation.y = transform.getRotation().y();
        pose.orientation.z = transform.getRotation().z();
//    printf("pose : x : %f, y : %f\n",pose.position.x,pose.position.y);
    }

    void setPoint(geometry_msgs::Point &point,const tf::StampedTransform transform){
        point.x = transform.getOrigin().x();
        point.y = transform.getOrigin().y();
        point.z = 0.1;
//    printf("point : x : %f, y : %f\n",point.x,point.y);
    }

    void stopRobot(const ros::Publisher& pub){
        geometry_msgs::Twist twist0, twist1;
        twist0.linear.x = twist0.linear.y = twist0.linear.z = 0.0;
        twist0.angular.x = twist0.angular.y = 0.0;
        twist0.angular.z = 0.1;

        twist1.linear.x = twist1.linear.y = twist1.linear.z = 0.0;
        twist1.angular.x = twist1.angular.y = twist1.angular.z = 0.0;

        for(int i = 0;i < 5;i++)
            pub.publish(twist0);
        // ros::Duration(0.2).sleep();
        for(int i = 0 ;i < 5;i++)
            pub.publish(twist1);
    }

    bool isRobotStop(const tf::StampedTransform &curr_transform, const geometry_msgs::Pose& last_pose,
                                   const double &pos_stop_threshold,const double &ang_stop_threshold,const int &index){
        double useless_pitch,useless_roll,curr_yaw,last_yaw;
        tf2::Transform last_transform;
        tf2::convert(last_pose,last_transform);
        curr_transform.getBasis().getEulerYPR(curr_yaw,useless_pitch,useless_roll);
        curr_yaw = angles::normalize_angle(curr_yaw);
        last_transform.getBasis().getEulerYPR(last_yaw,useless_pitch,useless_roll);
        last_yaw = angles::normalize_angle(last_yaw);
        double x_diff = fabs(curr_transform.getOrigin().x() - last_transform.getOrigin().x());
        double y_diff = fabs(curr_transform.getOrigin().y() - last_transform.getOrigin().y());
        double th_diff = fabs(curr_yaw - last_yaw);
//    if (index == 1){
//        ROS_INFO("x_diff : %f, y_diff : %f, th_diff : %f",x_diff,y_diff,th_diff);
//    }
        if (x_diff <= pos_stop_threshold && y_diff <= pos_stop_threshold && th_diff <= ang_stop_threshold)
            return true;
        else
            return false;
    }

    void calcTotalTime(const tf::StampedTransform &curr_transform, const geometry_msgs::Pose& last_pose,
                       double &time,ros::Time &start_time,ros::Time &end_time,
                       const double pos_stop_threshold,const double ang_stop_threshold,const int &index){
        bool stop = isRobotStop(curr_transform,last_pose,pos_stop_threshold,ang_stop_threshold,index);
//    if (index == 0){
//        ROS_INFO("ROBOT0 stop?%s",stop?"yes":"no");
//    }
        if (!stop && start_time.isZero())
            start_time = ros::Time::now();
        if (stop && !start_time.isZero()){
            end_time = ros::Time::now();
            time = (end_time - start_time).toSec();
            if (time >= 0.5){
                ROS_INFO("Robot%d runtime : %f",index,time);
            }
            start_time = end_time = ros::Time(0);
        }
    }
    void calcTotalTime(const tf::StampedTransform &curr_transform, const geometry_msgs::Pose &last_pose,
                       double &time, ros::Time &start_time, ros::Time &end_time, 
                       const MultiNavUtil::runstate *runstate, const int index){
        // std::cout<< "Time " << runstate->idx << ":" << "isstart," << runstate->isStart << "; isEnd," <<runstate->isEnd << "\n";
        if (runstate->isStart && start_time.isZero())
            start_time = ros::Time::now();
        if (runstate->isEnd && !start_time.isZero()){
            end_time = ros::Time::now();
            double tmp_time = (end_time - start_time).toSec();
            if (tmp_time >= MultiNavUtil::tiny_time && runstate->idx == index && (tmp_time - time) >= MultiNavUtil::tiny_time){
                time = tmp_time;
                ROS_INFO("Robot%d runtime : %f",runstate->idx, time);
            }
            start_time = end_time = ros::Time(0);
        }
    }

    void calcTotalDistance(const tf::StampedTransform &curr_transform, const geometry_msgs::Pose& last_pose,
                           const double pos_stop_threshold,const double ang_stop_threshold, double &total_distance,
                           const int index){
        bool stop = isRobotStop(curr_transform,last_pose,pos_stop_threshold,ang_stop_threshold,index);
        if (!stop){
            double x_diff = curr_transform.getOrigin().x() - last_pose.position.x;
            double y_diff = curr_transform.getOrigin().y() - last_pose.position.y;
            total_distance += sqrt(x_diff * x_diff + y_diff * y_diff);
        }
//    if (index == 1){
//        ROS_INFO("ROBOT0 has run for %f",total_distance);
//    }
        if (stop && total_distance != 0.0){
            ROS_INFO("Robot%d distance : %f",index,total_distance);
            total_distance = 0;
        }
    }

    void calcTotalDistance(const tf::StampedTransform &curr_transform,
                                         const geometry_msgs::Pose &last_pose, double &total_distance,
                                         const MultiNavUtil::runstate *runstate, const int index) {

        // std::cout << "Distance " <<runstate->idx << ":" << "isstart," << runstate->isStart << "; isEnd," <<runstate->isEnd << "\n";
        if (runstate->isStart){
            double x_diff = curr_transform.getOrigin().x() - last_pose.position.x;
            double y_diff = curr_transform.getOrigin().y() - last_pose.position.y;
            total_distance += sqrt(x_diff * x_diff + y_diff * y_diff);
        }
        if (runstate->isEnd && total_distance >= MultiNavUtil::tiny_offset && runstate->idx == index){
            ROS_INFO("Robot%d distance : %f",runstate->idx, total_distance);
            total_distance = 0;
        }
    }
}