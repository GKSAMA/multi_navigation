//
// Created by gk on 2022/2/21.
//

#include "eb_local_planner/conversions_and_types.h"

namespace eb_local_planner{

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

    bool transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                             costmap_2d::Costmap2DROS& costmap, const std::string& global_frame,
                             std::vector<geometry_msgs::PoseStamped>& transformed_plan, std::vector<int>& start_end_counts_from_end){
        const geometry_msgs::PoseStamped& plan_pose = global_plan[0];
        transformed_plan.clear();

        try {
            if (global_plan.empty()){
                ROS_ERROR("Receive plan with zero length");
                return false;
            }
            geometry_msgs::TransformStamped transform;
            transform = tf.lookupTransform(global_frame,ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp,
                                           plan_pose.header.frame_id);
            geometry_msgs::TransformStamped robot_pose;
            robot_pose.transform.rotation.w = 1;
            robot_pose.header.frame_id = costmap.getBaseFrameID();
            robot_pose.header.stamp = ros::Time();
            tf.transform(robot_pose,robot_pose,plan_pose.header.frame_id);

            double dist_threshold = std::max(
                    costmap.getCostmap()->getSizeInCellsX() / 2.0,
                    costmap.getCostmap()->getSizeInMetersY() / 2.0
                    );
            unsigned int i = 0;
            double sq_dist_threshold = dist_threshold * dist_threshold;
            double sq_dist = DBL_MAX;

            std::vector<int> start_end_count;
            start_end_count.assign(2,0);
            ROS_ASSERT((start_end_counts_from_end.at(0) > 0) && (start_end_counts_from_end.at(0) <= (int)global_plan.size()));
            i = (unsigned int)global_plan.size() - (unsigned int)start_end_counts_from_end.at(0);

            while(i<(unsigned int)global_plan.size() && sq_dist > sq_dist_threshold){
                double x_diff = robot_pose.transform.translation.x - global_plan[i].pose.position.x;
                double y_diff = robot_pose.transform.translation.y - global_plan[i].pose.position.y;
                sq_dist = x_diff * x_diff + y_diff * y_diff;

                if (sq_dist > sq_dist_threshold) i++;
                else start_end_count.at(0) = (int)(((unsigned int)global_plan.size()) - i);
            }

            tf2::Stamped<tf2::Transform> tf_pose, tf_transform;
            geometry_msgs::PoseStamped newer_pose;
            while(i<(unsigned int)global_plan.size() && sq_dist < sq_dist_threshold){
                double x_diff = robot_pose.transform.translation.x - global_plan[i].pose.position.x;
                double y_diff = robot_pose.transform.translation.y - global_plan[i].pose.position.y;
                sq_dist = x_diff * x_diff + y_diff * y_diff;
                const geometry_msgs::PoseStamped& pose = global_plan[i];
                tf2::convert(pose,tf_pose);
                tf2::convert(transform,tf_transform);
                tf_pose.setData(tf_transform * tf_pose);
                tf_pose.stamp_ = tf_transform.stamp_;
                tf_pose.frame_id_ = global_frame;
                tf2::toMsg(tf_pose,newer_pose);
                transformed_plan.push_back(newer_pose);

                start_end_count.at(1) = int (((unsigned int)global_plan.size()) - i);
                i++;
            }
            start_end_counts_from_end = start_end_count;
        }catch(tf2::LookupException& ex){
            ROS_ERROR("No Transform available Error: %s\n", ex.what());
            return false;
        }
        catch(tf2::ConnectivityException& ex){
            ROS_ERROR("Connectivity Error: %s\n", ex.what());
            return false;
        }
        catch(tf2::ExtrapolationException& ex){
            ROS_ERROR("Extrapolation Error: %s\n", ex.what());
            if (!global_plan.empty())
                ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());
            return false;
        }
        return true;
    }
    double getCircumscribedRadius(costmap_2d::Costmap2DROS& costmap){
        std::vector<geometry_msgs::Point> footprint(costmap.getRobotFootprint());
        double max_distance_sqr = 0;
        for(size_t i=0;i<footprint.size(); ++i){
            geometry_msgs::Point& p = footprint[i];
            double distance_sqr = p.x * p.x + p.y * p.y;
            if(distance_sqr > max_distance_sqr)
                max_distance_sqr = distance_sqr;
        }
        return sqrt(max_distance_sqr);
    }
}