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
#include "laser_geometry/laser_geometry.h"

#include "multi_turtlebot3_navigation/utils.h"
#define Dist2ObsPath "/home/gk/Documents/DataRecord/dist.data"
#define pose2Ddatapath "/home/gk/Documents/DataRecord/pose2Ddata.data"
#define veldatapath "/home/gk/Documents/DataRecord/veldata.data"

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
        ofs.open(Dist2ObsPath,std::ios::out|std::ios::app);
        ofs << time << " " << minrange << " " << rs->idx << std::endl;
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
        ofspose.open(pose2Ddatapath,std::ios::out|std::ios::app);
        ofsvel.open(veldatapath,std::ios::out|std::ios::app);
        ofspose << time << " " << pose2D.x << " " <<  pose2D.y << " " <<  pose2D.theta << " " << rs->waypoint << " " << idx << std::endl;
        ofsvel << time << " " << vel.linear.x << " " <<  vel.linear.y << " " <<  vel.angular.z << " " << rs->waypoint << " " << idx << std::endl;
        ofspose.close();
        ofsvel.close();
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv,"check_distance");
    ros::NodeHandle nh;
    int shmid;
    void *shared_memory = (void*)0;
    struct MultiNavUtil::runstate *rs;
    shmid = shmget((key_t)1234, sizeof(struct MultiNavUtil::runstate), 0666 | IPC_CREAT);
    shared_memory = shmat(shmid, (void*)0, 0);
    rs = (struct MultiNavUtil::runstate*)shared_memory;
    ros::Subscriber scansub0 = nh.subscribe<sensor_msgs::LaserScan>("/tb3_0/scan",100,boost::bind(&scanCB,_1,rs,0));
    ros::Subscriber scansub1 = nh.subscribe<sensor_msgs::LaserScan>("/tb3_1/scan",100,boost::bind(&scanCB,_1,rs,1));
    ros::Subscriber scansub2 = nh.subscribe<sensor_msgs::LaserScan>("/tb3_2/scan",100,boost::bind(&scanCB,_1,rs,2));
    ros::Subscriber footprintsub0 = nh.subscribe<geometry_msgs::PolygonStamped>
                                                    ("/tb3_0/move_base/global_costmap/footprint",
                                                    100,
                                                    boost::bind(&fpCB,_1,rs,0));
    ros::Subscriber footprintsub1 = nh.subscribe<geometry_msgs::PolygonStamped>
                                                    ("/tb3_1/move_base/global_costmap/footprint",
                                                    100,
                                                    boost::bind(&fpCB,_1,rs,1));
    ros::Subscriber footprintsub2 = nh.subscribe<geometry_msgs::PolygonStamped>
                                                    ("/tb3_2/move_base/global_costmap/footprint",
                                                    100,
                                                    boost::bind(&fpCB,_1,rs,2));
    ros::Subscriber odomsub0 = nh.subscribe<nav_msgs::Odometry>("/tb3_0/odom",100,boost::bind(&odomCB,_1,rs,0));
    ros::Subscriber odomsub1 = nh.subscribe<nav_msgs::Odometry>("/tb3_1/odom",100,boost::bind(&odomCB,_1,rs,1));
    ros::Subscriber odomsub2 = nh.subscribe<nav_msgs::Odometry>("/tb3_2/odom",100,boost::bind(&odomCB,_1,rs,2));
    ros::spin();
}