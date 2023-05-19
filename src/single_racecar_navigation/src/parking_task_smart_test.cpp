/*
 * @Author: Gong, Ke gk.kingj@gmail.com
 * @Date: 2022-11-22 21:16:50
 * @LastEditors: Gong, Ke gk.kingj@gmail.com
 * @LastEditTime: 2023-03-31 22:55:49
 * @FilePath: /src/single_racecar_navigation/src/task3.cpp
 * @Description: 
 * 
 * Copyright (c) 2022 by Gong, Ke gk.kingj@gmail.com, All Rights Reserved. 
 */
#include <fstream>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <sys/wait.h>
#include <tf2/utils.h>
#include <tf2/convert.h>
#include <sys/shm.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <multi_turtlebot3_navigation/utils.h>

#include <parking_task/parking.h>
#include <hybrid_a_star/hybrid_a_star_flow.h>
#include "single_racecar_navigation/ParkingSpaceInfo.h"
#include "single_racecar_navigation/gazebo_controller_manager.h"
#include "single_racecar_navigation/load_parameters.h"
#include "single_racecar_navigation/utils.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void odomCB(const nav_msgs::Odometry::ConstPtr& msg, geometry_msgs::PoseStamped* carState)
{
    carState->pose = msg->pose.pose;
    // std::cout << carState->pose.position.x << " " << carState->pose.position.y << " " << std::endl;
}

void targetCB(const single_racecar_navigation::ParkingSpaceInfo::ConstPtr& msg, 
              single_racecar_navigation::ParkingSpaceInfo *targetInfo,
              int *receiveTarget)
{
    single_racecar_navigation::ParkingSpaceInfo *tmp = new single_racecar_navigation::ParkingSpaceInfo();
    targetInfo->p1 = msg->p1;
    targetInfo->p2 = msg->p2;
    targetInfo->p3 = msg->p3;
    targetInfo->p4 = msg->p4;
    targetInfo->yaw = msg->yaw;
    targetInfo->parking_space_type = msg->parking_space_type;
    targetInfo->length = msg->length;
    targetInfo->width = msg->width;
    *receiveTarget = 1;
    std::cout << "receive target info!" << std::endl;
}
int main(int argc, char **argv)
{
    // init ROS
    ros::init(argc, argv, "parking_task_smart_test");
    ros::NodeHandle nh;

    // shared mem for data record
    void *shared_memory = (void*)0;
    struct MultiNavUtil::runstate *rs;
    int shmid;
    srand((unsigned int)getpid());
    shmid = shmget((key_t)9999,sizeof(struct MultiNavUtil::runstate),0666 | IPC_CREAT);
    if(shmid == -1){
        std::cout << "shmget failed!\n";
        exit(EXIT_FAILURE);
    }
    shared_memory = shmat(shmid,(void *)0,0);
    rs = (struct MultiNavUtil::runstate*)shared_memory;
    rs->isStart = false;
    rs->isEnd = false;
    rs->waypoint = 0;
    rs->idx = 0;

    // receive target information from Qt
    int *receiveTarget = new int(-1);
    single_racecar_navigation::ParkingSpaceInfo *targetInfo = new single_racecar_navigation::ParkingSpaceInfo();
    ros::Subscriber target_sub = 
        nh.subscribe<single_racecar_navigation::ParkingSpaceInfo>("/parking_position",
                                                                  1,
                                                                  boost::bind(&targetCB, _1, targetInfo, receiveTarget));
    std::string pathToPackage = ros::package::getPath("single_racecar_navigation");

    // load parking parameters
    SingleNavUtils::Configurations config(pathToPackage);
    
    std::cout << "create configurations finish" << std::endl;
    int parking_space_type = 0;
    float posx,posy,posz,posth;
    float vx = -0.3, vth = 0.785;
    std::string robotName="racecar_0";
    nh.getParam("pos_x", posx);
    nh.getParam("pos_y", posy);
    nh.getParam("pos_z", posz);
    nh.getParam("pos_yaw", posth);
    nh.getParam("robotName", robotName);
    nh.getParam("vel_x", vx);
    nh.getParam("vel_th", vth);
    nh.getParam("parking_space_type", parking_space_type);
    std::cout << *receiveTarget << std::endl;
    geometry_msgs::PoseStamped *carState = new geometry_msgs::PoseStamped();
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(std::string(robotName + "/odom"),1, boost::bind(&odomCB, _1, carState));
    std::cout << *receiveTarget << std::endl;

    // car position initialization
    ParkingTask ptask(nh,robotName);
    std::cout << *receiveTarget << std::endl;
    carState->pose.position.x = posx;
    carState->pose.position.y = posy;
    carState->pose.position.z = posz;
    tf2::Quaternion q;
    q.setRPY(0,0,posth);
    tf2::convert(q, carState->pose.orientation);    
    std::cout << *receiveTarget << std::endl;
    ptask.SetCarState(carState);

    // waiting for receiving target information
    std::cout << *receiveTarget << std::endl;
    if(*receiveTarget == -1){
        std::cout << "NOT RECEIVE TARGET INFORMATION!!!!!!!!!!!!!!" << std::endl;
    }
    while(*receiveTarget != 1){
        ros::spinOnce();
    }

    // load target information
    config.loadTargetInfo(targetInfo);
    
    // set parking space attribute
    ptask.SetParkingSpace(config.GetParkingSpace());
    ptask.SetCarParam(config.GetCarParam());
    ptask.SetSafeDistance(config.GetSafeDist());
    parking_space_type = config.GetParkingSpaceType();
    // std::cout << "robotName: " << robotName <<" pos_x=" << posx << " pos_y=" << posy << " pos_z=" << posz << " pos_yaw=" << posth << "parking_space_type=" << parking_space_type << std::endl;
    geometry_msgs::Pose target;
    
    // compute navigation pose
    switch (parking_space_type) {
    case 0:
        SingleNavUtil::ComputeVerticalTarget(config.GetParkingSpace(), config.GetCarParam(), target);
        break;
    case 1:
        SingleNavUtil::ComputeParallelTarget(config.GetParkingSpace(), config.GetCarParam(), target, M_PI_4, M_PI_4, 0.1);
        break;
    default:
        break;
    }
    std::cout << "target : x = " << target.position.x << " y = " << target.position.y << " z = " << target.position.z << " yaw = " << tf2::getYaw(target.orientation) << std::endl;
    // bool flag = ptask.VerticalTask(vx);

    // execute navigation
    bool succeed = false;
    MoveBaseClient ac("/" + robotName + "/move_base", true);
    // wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    std::cout << "move base client: " << "/"+robotName+"/move_base" << std::endl;
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose = target;
    std::cout << "sending goal: " << goal.target_pose.pose.position.x << " "<<goal.target_pose.pose.position.y << std::endl;
    
    // start record
    MultiNavUtil::setRunstate(rs,true,false,0,0);
    
    ac.sendGoal(goal);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        succeed = true;
        MultiNavUtil::setRunstate(rs,false,true,0,0);
    }
    if(!succeed){
        std::cout << "can't reach the target position, navigate failed!" << std::endl;
        ros::shutdown();
        return 0;
    }

    // geometry_msgs::PoseStamped goal;
    // goal.header.frame_id = "map";
    // goal.header.stamp = ros::Time::now();
    // goal.pose = target;

    // HybridAStarFlow kinodynamic_astar_flow(nh);
    // kinodynamic_astar_flow.init_pose_ = *carState;
    // kinodynamic_astar_flow.move_base_goal_ = goal;
    
    // Vec3d target_state(target.position.x, target.position.y, tf2::getYaw(target.orientation));
    // Vec3d current_state(carState->pose.position.x, carState->pose.position.y, tf2::getYaw(carState->pose.orientation));
    // ros::Rate rate(10);
    // while (ros::ok()) {
    //     kinodynamic_astar_flow.Run();

    //     ros::spinOnce();
    //     rate.sleep();
    // }

    // exectue parking task
    bool flag  = true;
    ros::Time timer = ros::Time::now();
    while(true){
        ros::Duration diff = ros::Time::now() - timer;
        // std::cout << diff << std::endl;
        if(diff >= ros::Duration(0.1)){
            break;
        }
        ros::spinOnce();
    }
    ptask.SetCarState(carState);
    std::cout << carState->pose.position.x << " " << carState->pose.position.y << " " << std::endl;
    switch (parking_space_type) {
    case 0:
        flag = ptask.VerticalTask(vx);
        break;
    case 1:
        flag = ptask.ParallelTask(vx);
        break;
    default:
        break;
    }
    if(flag){
        std::cout << "Parking task succeed !" << std::endl;
    } else {
        std::cout << "Parking task failed !" << std::endl;
    }
    // ptask.test(-0.5, 0.785);
    ros::shutdown();
    return 0;
}