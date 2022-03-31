//
// Created by gk on 2022/2/22.
//

#include "eb_local_planner/eb_visualization.h"

namespace eb_local_planner{
    EBVisualization::EBVisualization() :initialized_(false){}
    EBVisualization::EBVisualization(ros::NodeHandle& pn, costmap_2d::Costmap2DROS* costmap_ros) { initialize(pn,costmap_ros); }
    EBVisualization::~EBVisualization() {}

    void EBVisualization::initialize(ros::NodeHandle& pn, costmap_2d::Costmap2DROS* costmap_ros){
        if (!initialized_){
            pn.param("marker_lifetime", marker_lifetime_, 0.5);
            one_bubble_pub_ = pn.advertise<visualization_msgs::Marker>("eb_visualization",1);
            bubble_pub_ = pn.advertise<visualization_msgs::MarkerArray>("eb_visualization_array", 1);
            costmap_ros_ = costmap_ros;

            initialized_ = true;
        }else{
            ROS_WARN("Trying to initialize already initialized visualization, doing nothing.");
        }
    }
    void EBVisualization::reconfigure(EBPlannerConfig& config){
        marker_lifetime_ = config.marker_lifetime;
    }
    void EBVisualization::publishBand(std::string marker_name_space, std::vector<Bubble> band){
        if(!initialized_){
            ROS_ERROR("Visualization not yet initialized, please call initialize() before using visualization");
            return;
        }
        visualization_msgs::MarkerArray eb_msg;
        eb_msg.markers.resize(band.size());
        visualization_msgs::MarkerArray eb_heading_msg;
        eb_heading_msg.markers.resize(band.size());
        std::string marker_heading_name_space = marker_name_space;
        marker_heading_name_space.append("_heading");

        for (int i = 0; i < ((int)band.size()); ++i) {
            bubbleToMarker(band[i], eb_msg.markers[i], marker_name_space, i ,green);
//            bubbleHeadingToMarker(band[i], eb_heading_msg.markers[i], marker_name_space, i ,green);
        }
        bubble_pub_.publish(eb_msg);
//        bubble_pub_.publish(eb_heading_msg);
    }
    void EBVisualization::publishBubble(std::string marker_name_space, int marker_id, Bubble bubble){
        if(!initialized_){
            ROS_ERROR("Visualization not yet initialized, please call initialize() before using visualization");
            return;
        }
        visualization_msgs::Marker bubble_msg;
        bubbleToMarker(bubble,bubble_msg,marker_name_space,marker_id,green);
        one_bubble_pub_.publish(bubble_msg);
    }
    void EBVisualization::publishBubble(std::string marker_name_space, int marker_id, Color marker_color,Bubble bubble){
        if(!initialized_){
            ROS_ERROR("Visualization not yet initialized, please call initialize() before using visualization");
            return;
        }
        visualization_msgs::Marker bubble_msg;
        bubbleToMarker(bubble,bubble_msg,marker_name_space,marker_id,marker_color);
        one_bubble_pub_.publish(bubble_msg);
    }
    void EBVisualization::publishForceList(std::string marker_name_space, std::vector<geometry_msgs::WrenchStamped> forces, std::vector<Bubble> band){
        if(!initialized_){
            ROS_ERROR("Visualization not yet initialized, please call initialize() before using visualization");
            return;
        }
        visualization_msgs::MarkerArray forces_msg;
        forces_msg.markers.resize(forces.size());

        Color marker_color = green;
        if (marker_name_space.compare("internal_forces") == 0)
            marker_color = blue;
        if (marker_name_space.compare("external_forces") == 0)
            marker_color = red;
        if (marker_name_space.compare("resulting_forces") == 0)
            marker_color = green;
        for (int i = 0; i < ((int)forces.size()); ++i) {
            forceToMarker(forces[i], band[i].center.pose, forces_msg.markers[i], marker_name_space,i,marker_color);
        }
        bubble_pub_.publish(forces_msg);
    }
    void EBVisualization::publishForce(std::string marker_name_space, int id, Color marker_color, geometry_msgs::WrenchStamped force, Bubble bubble){
        if(!initialized_){
            ROS_ERROR("Visualization not yet initialized, please call initialize() before using visualization");
            return;
        }
        visualization_msgs::Marker force_msg;
        forceToMarker(force,bubble.center.pose,force_msg,marker_name_space,id,marker_color);
        one_bubble_pub_.publish(force_msg);
    }

    void EBVisualization::bubbleToMarker(Bubble bubble, visualization_msgs::Marker& marker,std::string marker_name_space, int marker_id, Color marker_color){
        geometry_msgs::Pose2D tmp_pose2d;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = bubble.center.header.frame_id;

        marker.ns = marker_name_space;
        marker.id = marker_id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = bubble.center.pose;
        PoseToPose2D(bubble.center.pose,tmp_pose2d);
        marker.pose.position.z = 0;
        marker.scale.x = 2.0*bubble.expansion;
        marker.scale.y = 2.0*bubble.expansion;
        marker.scale.z = 2.0*bubble.expansion;
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        switch (marker_color) {
            case red:   {marker.color.r = 1.0f; break;}
            case green: {marker.color.g = 1.0f; break;}
            case blue:  {marker.color.b = 1.0f; break;}
        }
        marker.color.a = 0.75;
        marker.lifetime = ros::Duration(marker_lifetime_);
    }
    void EBVisualization::bubbleHeadingToMarker(Bubble bubble, visualization_msgs::Marker marker, std::string marker_name_space, int marker_id, Color marker_color){
        geometry_msgs::Pose2D tmp_pose2d;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = bubble.center.header.frame_id;

        marker.ns = marker_name_space;
        marker.id = marker_id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = bubble.center.pose;
        PoseToPose2D(bubble.center.pose,tmp_pose2d);
        marker.pose.position.z = tmp_pose2d.theta* getCircumscribedRadius(*costmap_ros_);
        marker.scale.x = 2.0*bubble.expansion;
        marker.scale.y = 2.0*bubble.expansion;
        marker.scale.z = 2.0*bubble.expansion;
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        switch (marker_color) {
            case red:   {marker.color.r = 1.0f; break;}
            case green: {marker.color.g = 1.0f; break;}
            case blue:  {marker.color.b = 1.0f; break;}
        }
        marker.color.a = 0.75;
        marker.lifetime = ros::Duration(marker_lifetime_);
    }
    void EBVisualization::forceToMarker(geometry_msgs::WrenchStamped wrench, geometry_msgs::Pose wrench_origin, visualization_msgs::Marker& marker,std::string marker_name_space, int marker_id, Color marker_color){
        geometry_msgs::Pose2D tmp_pose2d;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = wrench.header.frame_id;

        marker.ns = marker_name_space;
        marker.id = marker_id;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = wrench_origin.position;
        PoseToPose2D(wrench_origin,tmp_pose2d);
        marker.pose.position.z = tmp_pose2d.theta* getCircumscribedRadius(*costmap_ros_);

        if ((wrench.wrench.force.x!=0)||(wrench.wrench.force.y!=0)||(wrench.wrench.force.z!=0)){
            Eigen::Vector3d x_axis(1.0,0.0,0.0);
            Eigen::Vector3d target_vec(wrench.wrench.force.x,wrench.wrench.force.y,wrench.wrench.force.z/
                                                                                   getCircumscribedRadius(*costmap_ros_));
            Eigen::Vector3d rotation_axis(1.0,0.0,0.0);
            double rotation_angel = 0.0;
            x_axis.normalize();
            if(!(x_axis==target_vec)){
                rotation_axis = x_axis.cross(target_vec);
                rotation_angel = x_axis.dot(target_vec);
                rotation_angel = acos(rotation_angel);
            }
            rotation_axis.normalize();
            const double s = sin(rotation_angel/2);
            const double c = cos(rotation_angel/2);
            Eigen::Quaterniond rotate_quat(c,s*rotation_axis.x(), s*rotation_axis.y(),s*rotation_axis.z());
            geometry_msgs::Quaternion orientation_msg;
            tf2::convert(rotate_quat, orientation_msg);
            marker.pose.orientation = orientation_msg;
            double scale = sqrt((wrench.wrench.force.x*wrench.wrench.force.x)+(wrench.wrench.force.y*wrench.wrench.force.y)+
                    ((wrench.wrench.force.z/
                     getCircumscribedRadius(*costmap_ros_)) * (wrench.wrench.force.z/
                                                             getCircumscribedRadius(*costmap_ros_))));
            marker.scale.x = scale;
            marker.scale.y = scale;
            marker.scale.z = scale;
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            switch (marker_color) {
                case red:   {marker.color.r = 1.0f; break;}
                case green: {marker.color.g = 1.0f; break;}
                case blue:  {marker.color.b = 1.0f; break;}
            }
            marker.color.a = 1.25;
        }else{
            marker.pose.orientation = wrench_origin.orientation;
            marker.scale.x = 0.0;
            marker.scale.y = 0.0;
            marker.scale.z = 0.0;
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.0;
        }
        marker.lifetime = ros::Duration(marker_lifetime_);
    }
}