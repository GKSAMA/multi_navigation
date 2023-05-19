/*
 * @Author: Gong, Ke gk.kingj@gmail.com
 * @Date: 2023-02-10 14:40:44
 * @LastEditors: Gong, Ke gk.kingj@gmail.com
 * @LastEditTime: 2023-04-24 15:25:20
 * @FilePath: /src/single_racecar_navigation/src/data_record.cpp
 * @Description: 
 * 
 * Copyright (c) 2023 by Gong, Ke gk.kingj@gmail.com, All Rights Reserved. 
 */
#include <ros/ros.h>
#include "tf/transform_listener.h"
#include "tf2/convert.h"
#include "tf2/transform_datatypes.h"
#include "tf2/utils.h"
#include "sys/shm.h"
#include "fstream"
#include "iostream"
#include "boost/bind.hpp"

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include <multi_turtlebot3_navigation/utils.h>
#define SavePath "/home/gk/Documents/DataRecord/smart"
// #define AlgorithmType "teb2"
#define AlgorithmType "dwa"

#define Dist2ObsPath "minobsdist.data"
#define pose2Ddatapath "pose2Ddata.data"
#define veldatapath "veldata.data"
#define dist2goal "dist2goal.data"
#define commandRate "commandRate2.data"

void scanCB(const sensor_msgs::LaserScan::ConstPtr& scan, MultiNavUtil::runstate *rs,int idx){
    if(rs->isStart && rs->idx == idx){
        double time = scan->header.stamp.toSec();
        double minrange = scan->range_max;
        for(int i = 0; i < scan->ranges.size();i++){
            // std::cout << scan->ranges[i] << " ";
            if(minrange > scan->ranges[i])
                minrange = scan->ranges[i];
        }
        std::ofstream ofs;
        ofs.open(std::string(SavePath) + "/" + AlgorithmType + "/" + std::string(Dist2ObsPath),std::ios::out|std::ios::app);
        ofs << time << " " << minrange << std::endl;
        ofs.close();
    }
}

void fpCB(const geometry_msgs::PolygonStamped::ConstPtr& polygonMsg,MultiNavUtil::runstate *rs,int idx){
    if (rs->isStart && rs->idx == idx){
        double time = polygonMsg->header.stamp.toSec();
        double x = (polygonMsg->polygon.points[0].x+
                    polygonMsg->polygon.points[1].x+
                    polygonMsg->polygon.points[2].x+
                    polygonMsg->polygon.points[3].x)/4;
        double y = (polygonMsg->polygon.points[0].y+
                    polygonMsg->polygon.points[1].y+
                    polygonMsg->polygon.points[2].y+
                    polygonMsg->polygon.points[3].y)/4;
        
    }
    
}

void odomCB(const nav_msgs::Odometry::ConstPtr& odometry, MultiNavUtil::runstate* rs,int idx){
    if(rs->isStart && rs->idx == idx){
        double time = odometry->header.stamp.toSec();
        double yaw = tf::getYaw(odometry->pose.pose.orientation);
        geometry_msgs::Twist vel = odometry->twist.twist;
        geometry_msgs::Pose pose = odometry->pose.pose;
        geometry_msgs::Pose2D pose2D;
        MultiNavUtil::PoseToPose2D(pose,pose2D);

        std::ofstream ofspose,ofsvel;
        ofspose.open(std::string(SavePath) + "/" + AlgorithmType + "/" + std::string(pose2Ddatapath),std::ios::out|std::ios::app);
        ofsvel.open(std::string(SavePath) + "/" + AlgorithmType + "/" + std::string(veldatapath),std::ios::out|std::ios::app);
        ofspose << time << " " << pose2D.x << " " <<  pose2D.y << " " <<  pose2D.theta << std::endl;
        ofsvel << time << " " << vel.linear.x << " " <<  vel.linear.y << " " <<  vel.angular.z << std::endl;
        ofspose.close();
        ofsvel.close();
    }
}

void pathCB(const nav_msgs::Path::ConstPtr& path, MultiNavUtil::runstate *rs,int idx){
    if(rs->isStart && rs->idx == idx){
        double time = path->header.stamp.toSec();
        double sumdist = 0;
        for(int i = 1; i < path->poses.size(); ++i){
            sumdist += sqrt(pow((path->poses[i].pose.position.x - path->poses[i - 1].pose.position.x), 2) + 
                            pow((path->poses[i].pose.position.y - path->poses[i-1].pose.position.y), 2));
        }
        std::ofstream ofglobaldist;
        ofglobaldist.open(std::string(SavePath) + "/" + AlgorithmType + "/" + std::string(dist2goal),std::ios::out|std::ios::app);
        ofglobaldist << time << " " << sumdist << std::endl;
        ofglobaldist.close();
    }
}

void cmdCB(const geometry_msgs::Point::ConstPtr& msg, MultiNavUtil::runstate *rs, int idx){
    // if(rs->isStart && rs->idx == idx){
        std::ofstream ofcmdrate;
        ofcmdrate.open(std::string(SavePath) + "/" + AlgorithmType + "/" + std::string(commandRate),std::ios::out|std::ios::app);
        std::cout << "command rate = " << msg->x << std::endl;
        ofcmdrate << msg->x << std::endl;
        ofcmdrate.close();
    // }
}

int main(int argc, char** argv){
    ros::init(argc, argv,"record_smart_data");
    ros::NodeHandle nh;
    int shmid;
    void *shared_memory = (void*)0;
    struct MultiNavUtil::runstate *rs;
    shmid = shmget((key_t)9999, sizeof(struct MultiNavUtil::runstate), 0666 | IPC_CREAT);
    shared_memory = shmat(shmid, (void*)0, 0);
    rs = (struct MultiNavUtil::runstate*)shared_memory;
    // reset shared memory
    rs->idx = -1;
    // ros::Subscriber scansub0 = nh.subscribe<sensor_msgs::LaserScan>("/smart_0/scan",100,boost::bind(&scanCB,_1,rs,0));
    // ros::Subscriber footprintsub0 = nh.subscribe<geometry_msgs::PolygonStamped>
    //                                                 ("/smart_0/move_base/global_costmap/footprint",
    //                                                 100,
    //                                                 boost::bind(&fpCB,_1,rs,0));
    // ros::Subscriber odomsub0 = nh.subscribe<nav_msgs::Odometry>("/smart_0/odom",100,boost::bind(&odomCB,_1,rs,0));

    // ros::Subscriber globalpathsub0 = nh.subscribe<nav_msgs::Path>("/smart_0/move_base/SmoothGlobalPlanner/smooth_plan", 100,boost::bind(&pathCB, _1, rs, 0));
    ros::Subscriber commandratesub0 = nh.subscribe<geometry_msgs::Point>("/smart_0/move_base/AckermannPlannerROS/command_rate", 100, boost::bind(&cmdCB, _1, rs, 0));
    std::cout << "start record!" << std::endl;
    ros::spin();
}