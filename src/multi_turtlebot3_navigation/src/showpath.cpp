#include <ros/ros.h>
#include "tf/transform_listener.h"
#include "tf2/convert.h"
#include "tf2/transform_datatypes.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/Marker.h"
#include "angles/angles.h"
#include "sys/shm.h"
#include "fstream"
#include "iostream"
#include "multi_turtlebot3_navigation/utils.h"

int main(int argc,char **argv){
    ros::init(argc, argv,"showpath");
    ros::NodeHandle n;
    ros::Publisher tb0_marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker0", 10);
    ros::Publisher tb1_marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker1", 10);
    ros::Publisher tb2_marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker2", 10);
    ros::Rate r(10);
    tf::TransformListener listener;
    void *shared_memory = (void*)0;
    struct MultiNavUtil::runstate *rs;
    char buffer[BUFSIZ];
    int shmid;
    shmid = shmget((key_t)1234, sizeof(struct MultiNavUtil::runstate), 0666 | IPC_CREAT);
    shared_memory = shmat(shmid, (void*)0, 0);
    rs = (struct MultiNavUtil::runstate*)shared_memory;

    while (!ros::ok()){
        r.sleep();
    }
// robot0
    visualization_msgs::Marker points0, line_strip0;
    ros::Time start0,end0;
    points0.header.frame_id = line_strip0.header.frame_id = "/map";
    points0.header.stamp = line_strip0.header.stamp = ros::Time::now();
    points0.ns = line_strip0.ns = "showpath";
    points0.action = line_strip0.action = visualization_msgs::Marker::ADD;
    points0.pose.orientation.w = line_strip0.pose.orientation.w = 1.0;
    
    points0.id = 0;
    line_strip0.id = 1;
    points0.type = visualization_msgs::Marker::POINTS;
    line_strip0.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip0.scale.x = 0.05;
    line_strip0.color.b = 1.0;
    line_strip0.color.a = 1.0;
// robot1
    visualization_msgs::Marker points1, line_strip1;
    ros::Time start1,end1;
    points1.header.frame_id = line_strip1.header.frame_id = "/map";
    points1.header.stamp = line_strip1.header.stamp = ros::Time::now();
    points1.ns = line_strip1.ns = "showpath";
    points1.action = line_strip1.action = visualization_msgs::Marker::ADD;
    points1.pose.orientation.w = line_strip1.pose.orientation.w = 1.0;

    points1.id = 0;
    line_strip1.id = 1;
    points1.type = visualization_msgs::Marker::POINTS;
    line_strip1.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip1.scale.x = 0.05;
    line_strip1.color.g = 1.0;
    line_strip1.color.a = 1.0;
// robot2
    visualization_msgs::Marker points2, line_strip2;
    ros::Time start2,end2;
    points2.header.frame_id = line_strip2.header.frame_id = "/map";
    points2.header.stamp = line_strip2.header.stamp = ros::Time::now();
    points2.ns = line_strip2.ns = "showpath";
    points2.action = line_strip2.action = visualization_msgs::Marker::ADD;
    points2.pose.orientation.w = line_strip2.pose.orientation.w = 1.0;

    points2.id = 0;
    line_strip2.id = 1;
    points2.type = visualization_msgs::Marker::POINTS;
    line_strip2.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip2.scale.x = 0.05;
    line_strip2.color.r = 1.0;
    line_strip2.color.a = 1.0;

    float x0(0), y0(0),x1(0), y1(0),x2(0), y2(0);
    int cnt0(0),cnt1(0),cnt2(0);
    double dist0(0.0),dist1(0.0),dist2(0.0);
    start0 = start1 = start2 = ros::Time(0);
    end0 = end1 = end2 = ros::Time(0);
    double time0(0.0),time1(0.0),time2(0.0);
    geometry_msgs::Point p0,p1,p2;
    geometry_msgs::Pose pose0,pose1,pose2;
    float pos_stop_threshold = 0.0001,th_stop_threshold = 0.001;

    tf::StampedTransform transform0, transform1, transform2;
    ros::Duration(1.0).sleep();
    try {
        listener.lookupTransform("/map", "tb3_0/base_footprint", ros::Time(0), transform0);
        listener.lookupTransform("/map", "tb3_1/base_footprint", ros::Time(0), transform1);
        listener.lookupTransform("/map", "tb3_2/base_footprint", ros::Time(0), transform2);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    MultiNavUtil::setPoint(p0,transform0);
    MultiNavUtil::setPoint(p1,transform1);
    MultiNavUtil::setPoint(p2,transform2);
    MultiNavUtil::setPose(pose0,transform0);
    MultiNavUtil::setPose(pose1,transform1);
    MultiNavUtil::setPose(pose2,transform2);

    while (ros::ok()){
        tf::StampedTransform transform0, transform1, transform2;
        try {
            listener.lookupTransform("/map", "tb3_0/base_footprint", ros::Time(0), transform0);
            listener.lookupTransform("/map", "tb3_1/base_footprint", ros::Time(0), transform1);
            listener.lookupTransform("/map", "tb3_2/base_footprint", ros::Time(0), transform2);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        rs = (struct MultiNavUtil::runstate*)shared_memory;
        // std::cout << rs->idx << ":" << "isstart," << rs->isStart << "; isEnd," <<rs->isEnd << "\n";
        // ROS_INFO("Runstate: isStart:%d; isEnd:%d, idx:%d",runstate.isStart,runstate.isEnd,runstate.idx);
//        MultiNavUtil::calcTotalTime(transform0,pose0,time0,start0,end0,pos_stop_threshold,th_stop_threshold,0);
//        MultiNavUtil::calcTotalDistance(transform0,pose0,pos_stop_threshold,th_stop_threshold,dist0,0);
        MultiNavUtil::calcTotalTime(transform0,pose0,time0,start0,end0,rs,0);
        MultiNavUtil::calcTotalDistance(transform0,pose0,dist0,rs,0);
        MultiNavUtil::setPose(pose0,transform0);
        MultiNavUtil::setPoint(p0,transform0);
        if(cnt0 > 1){line_strip0.points.push_back(p0);}
        else cnt0++;
        tb0_marker_pub.publish(line_strip0);

//        MultiNavUtil::calcTotalTime(transform1,pose1,time1,start1,end1,pos_stop_threshold,th_stop_threshold,1);
//        MultiNavUtil::calcTotalDistance(transform1,pose1,pos_stop_threshold,th_stop_threshold,dist1,1);
        MultiNavUtil::calcTotalTime(transform1,pose1,time1,start1,end1,rs,1);
        MultiNavUtil::calcTotalDistance(transform1,pose1,dist1,rs,1);
        MultiNavUtil::setPose(pose1,transform1);
        MultiNavUtil::setPoint(p1,transform1);
        if(cnt1 > 1){line_strip1.points.push_back(p1);}
        else cnt1++;
        tb1_marker_pub.publish(line_strip1);

//        MultiNavUtil::calcTotalTime(transform2,pose2,time2,start2,end2,pos_stop_threshold,th_stop_threshold,2);
//        MultiNavUtil::calcTotalDistance(transform2,pose2,pos_stop_threshold,th_stop_threshold,dist2,2);
        MultiNavUtil::calcTotalTime(transform2,pose2,time2,start2,end2,rs,2);
        MultiNavUtil::calcTotalDistance(transform2,pose2,dist2,rs,2);
        MultiNavUtil::setPose(pose2,transform2);
        MultiNavUtil::setPoint(p2,transform2);
        if(cnt2 > 1){line_strip2.points.push_back(p2);}
        else cnt2++;
        tb2_marker_pub.publish(line_strip2);
        r.sleep();
    }

}