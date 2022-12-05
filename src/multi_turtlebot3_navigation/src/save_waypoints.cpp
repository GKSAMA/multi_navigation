//
// Created by gk on 2022/3/7.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "iostream"
#include "vector"
#include "fstream"
#include "signal.h"
using namespace std;
#define path "/home/gk/Documents/multi_turtlebot3_navigation/src/single_racecar_navigation/config/parking.data"
// #define path "/home/gk/Documents/multi_turtlebot3_navigation/src/multi_turtlebot3_navigation/config/waypoints.data"
// #define path "/home/gk/Documents/multi_turtlebot3_navigation/src/multi_turtlebot3_navigation/config/humanwaypoints.data"

void save_Pose(vector<double> &p){
    ofstream ofs;
    ofs.open(path,ios::out|ios::app);
    ofs << p[0] << " " << p[1] << " " << p[2] << " " << p[3] <<endl;
    ofs.close();
    cout << "Save Waypoint succeed!" << endl;
}

void cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    cout << "get waypoint" <<endl;
    vector<double> pose;
    pose.push_back(msg->pose.position.x);
    pose.push_back(msg->pose.position.y);
    pose.push_back(msg->pose.orientation.w);
    pose.push_back(msg->pose.orientation.z);
    save_Pose(pose);
    pose.clear();
}

int main(int argc, char** argv){
    ros::init(argc,argv,"get_pose");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/racecar/move_base_simple/goal", 1000, cb);
    ros::spin();
    return 0;
}