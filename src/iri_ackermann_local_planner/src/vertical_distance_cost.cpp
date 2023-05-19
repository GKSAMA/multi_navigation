#include "vertical_distance_cost.h"
#include <angles/angles.h>
#include <fuzzy_controller/my_fuzzy_controller.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <queue>
#include <climits>
#include <sys/shm.h>
#include <boost/bind.hpp>

#define MAP_INDEX(map, i, j) ((i) + (j) * map.size_x)
#define MAP_WXGX(map, i) (map.origin_x + (i - map.size_x / 2) * map.scale)
#define MAP_WYGY(map, i) (map.origin_y + (i - map.size_y / 2) * map.scale)

void ScanCB(const sensor_msgs::LaserScan::ConstPtr& scan, double *minObsDist)
{
    double minrange = scan->range_max;
    for(int i = 0; i < scan->ranges.size();i++){
        // std::cout << scan->ranges[i] << " ";
        if(minrange > scan->ranges[i])
            minrange = scan->ranges[i];
    }
    *minObsDist = minrange;
}

namespace base_local_planner{
    VerticalDistanceCostFunction::VerticalDistanceCostFunction(costmap_2d::Costmap2D* costmap)
        : costmap_(costmap)
    {
        if(costmap != NULL){
            world_model_ = new CostmapModel(*costmap_);
        }
        void *shared_memory = (void*)0;
        int shmid;
        // shmid = shmget((key_t)0x4321, 0, 0);
        // if(shmid != -1){
        //     shmctl(shmid, IPC_RMID, 0);
        // }
        shmid = shmget((key_t)0x9876, sizeof(CostState), 0666 | IPC_CREAT);
        if(shmid == -1){
            std::cout << "Vertical shmget failed..." << std::endl;
        }
        shared_memory = shmat(shmid, NULL, 0);
        if(shared_memory == (void*)-1){
            std::cout << "Vertical shmat failed..." << std::endl;
        }
        costState = (CostState*)shared_memory;
        costState->init();
    }


    VerticalDistanceCostFunction::~VerticalDistanceCostFunction()
    {
        if(costmap_ != NULL){
            delete world_model_;
        }
    }

    bool VerticalDistanceCostFunction::prepare(){
        return true;
    }

    void VerticalDistanceCostFunction::SetNodeHandle(ros::NodeHandle& n) 
    { 
        nh = &n; 
        targetLinePub = nh->advertise<visualization_msgs::Marker>("target_line", 10);
        targetPosePub = nh->advertise<visualization_msgs::Marker>("target_pose", 10);
        scanSub = nh->subscribe<sensor_msgs::LaserScan>("/smart_0/scan", 100, boost::bind(ScanCB, _1, &min_obs_dist_));
    }

    void VerticalDistanceCostFunction::SetTargetPose(geometry_msgs::PoseStamped target){
        target_pose = target;

        // visualization in rviz
        visualization_msgs::Marker points, line;
        line.header.frame_id = "/map";
        line.header.stamp = ros::Time::now();
        line.ns = "targetLine";
        line.action = visualization_msgs::Marker::ADD;
        line.pose.orientation.w = 1.0;

        line.id = 1;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.scale.x = 0.1;
        line.color.a = 1.0;
        line.color.r = 1.0;
        line.color.g = 0.0;
        line.color.b = 0.0;

        double x,y,th;
        x = target_pose.pose.position.x; 
        y = target_pose.pose.position.y;
        tf2::Transform pose_tf;
        tf2::convert(target_pose.pose,pose_tf);
        double useless_pitch, useless_roll;
        pose_tf.getBasis().getEulerYPR(th, useless_pitch, useless_roll);
        th = angles::normalize_angle(th);
        double k,b;
        k = std::tan(th);
        b = y - k * x;
        geometry_msgs::Point p;
        double step = 0.05;
        for(int i = 0; i < 100; ++i){
            p.x = x + i * step;
            p.y = b + k * p.x;
            line.points.push_back(p);
        }
        for(int i = 0; i < 100; ++i){
            p.x = x - i * step;
            p.y = b + k * p.x;
            line.points.push_back(p);
        }
        targetLinePub.publish(line);
        
        visualization_msgs::Marker poseMarker;
        poseMarker.header.frame_id = "/map";
        poseMarker.ns = "targetPose";
        poseMarker.action = visualization_msgs::Marker::ADD;
        poseMarker.pose.orientation.w = 1.0;
        poseMarker.id = 3;
        poseMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        poseMarker.color.a = 1.0;
        poseMarker.scale.z = 0.3;
        poseMarker.color.r = 0.0;
        poseMarker.color.g = 1.0;
        poseMarker.color.b = 0.5;
        poseMarker.pose.position.x = x;
        poseMarker.pose.position.y = y;
        poseMarker.text = std::to_string(x) + "," + std::to_string(y);
        targetPosePub.publish(poseMarker);
    }

    void VerticalDistanceCostFunction::SetDesiredDirectionScale(double scale) {
        des_dir_scale_ = scale;
    }

    void VerticalDistanceCostFunction::ComputeTurnRadius(double wheelbase, double max_steer_angle)
    {
        min_turn_radius_ = sqrt(wheelbase * wheelbase / 4 + wheelbase * wheelbase / (tan(max_steer_angle) * tan(max_steer_angle)));
        // cout << 
        std::cout << "min turn radius : " << min_turn_radius_ << std::endl;
    }

    double VerticalDistanceCostFunction::ComputeMinObstacleDistance(int x, int y)
    {
        map_inf map_info;
        map_info.size_x = costmap_->getSizeInCellsX();
        map_info.size_y = costmap_->getSizeInCellsY();
        map_info.scale = costmap_->getResolution();
        map_info.origin_x = costmap_->getOriginX();
        map_info.origin_y = costmap_->getOriginY();

        cout << "map size: width : " << map_info.size_x << "  height : " << map_info.size_y << std::endl;
        std::cout << "origin x : " << map_info.origin_x << " origin y : " << map_info.origin_y << std::endl;
        // std::cout << " resolution : " << map_info.scale << std::endl;
        // min_obstacle_dist_ = INT_MAX;
        std::vector<bool> visited(map_info.size_x * map_info.size_y, false);
        std::vector<std::pair<int,int>> dir = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
        std::queue<std::pair<int,int>> qu;
        qu.push(std::make_pair((x - map_info.origin_x) / map_info.scale, (y - map_info.origin_y) / map_info.scale));
        visited[MAP_INDEX(map_info, (x - map_info.origin_x) / map_info.scale, (y - map_info.origin_y) / map_info.scale)] = true;
        // int count  = 10000;
        while (!qu.empty()){
            int levelNum = qu.size();
            for(int i = 0; i < levelNum; ++i){
                int curx = qu.front().first;
                int cury = qu.front().second;
                // std::cout << "cur x : " << curx << " cur y : " << cury << " cur cost : " << int(costmap_->getCost(curx, cury)) <<std::endl;
                qu.pop();
                if(costmap_->getCost(curx, cury) >= 240){

                    return std::sqrt(pow(x - (curx * map_info.scale + map_info.origin_x), 2) + 
                                     pow(y - (cury * map_info.scale + map_info.origin_y), 2));
                }
                for(int j = 0; j < 4; ++j){
                    int newx = curx + dir[j].first;
                    int newy = cury + dir[j].second;
                    if(newx < map_info.size_x && newx >= 0 && newy < map_info.size_y && newy >= 0 && !visited[MAP_INDEX(map_info, newx, newy)]){
                        visited[MAP_INDEX(map_info, newx, newy)] = true;
                        qu.push(std::make_pair(newx, newy));
                    }
                }
            }
        }
        return INT_MAX;
    }

    double VerticalDistanceCostFunction::scoreTrajectory(Trajectory &traj){
        // std::cout << "socre the vertical cost....." << std::endl;
        ros::spinOnce();
        double eps = 1e-4;
        double px, py, pth;
        int finalIndex = traj.getPointsSize() - 1;
        traj.getPoint(finalIndex, px, py, pth);
        costState->Traj_x = px;
        costState->Traj_y = py;
        double tx, ty, tth;
        tx = target_pose.pose.position.x; 
        ty = target_pose.pose.position.y;
        tf2::Transform pose_tf;
        tf2::convert(target_pose.pose,pose_tf);
        double useless_pitch, useless_roll;
        pose_tf.getBasis().getEulerYPR(tth, useless_pitch, useless_roll);
        tth = angles::normalize_angle(tth);
        tth = tf2::getYaw(target_pose.pose.orientation);
        double k, b, dl;
        if(abs(tth - M_PI / 2) < eps || abs(tth + M_PI / 2) < eps || abs(tth - 3 * M_PI / 2) < eps || abs(tth + 3 * M_PI / 2) < eps){
            dl = abs(tx - px);
        } else {
            k = std::tan(tth);
            b = ty - k * tx;
            dl = std::abs(k * px - py + b) / std::sqrt(k * k + 1 * 1);
        }
        // std::cout << "distance between vertical direction: " << dl << std::endl;
        double dMax = dl;
        double dMin = dl;
        for(unsigned int i = 0; i < traj.getPointsSize(); ++i){
            traj.getPoint(i, px, py, pth);
            double curDistance = std::abs(k * px - py + b) / std::sqrt(k * k + 1 * 1);
            dMax = std::max(dMax, curDistance);
            dMin = std::min(dMin, curDistance);
        }
        // double distVert = 1 - (dl / dMax);
        double distVert = dl / dMax;
        int subsetNum = 2;
        int inputNum = 3;
        int ruleNum = 6;
        double n1 = 0.01, n2 = 0.05, n3 = 0.5;
        traj.getPoint(0, px, py, pth);
        // std::cout << "vertical cost cur position x = " << vertCostState->traj_x << "  cur position y = " << vertCostState->traj_y << std::endl;
        // double MinObsDist = ComputeMinObstacleDistance(px,py);
        // std::cout << "MinObsDist : " << targetMinObsDist;
        // std::cout << "MinObsDist : " << min_obs_dist_;

        // VERTICAL COST
        // std::vector<double> inputData = {dl, abs(abs(tth) - abs(pth)), targetMinObsDist};
        std::vector<double> inputData = {dl, abs(abs(tth) - abs(pth)), min_obs_dist_};
        // std::cout << "inputData: ";
        // for(auto data : inputData){
        //     std::cout << data << " ";
        // }
        // std::cout << std::endl;
        std::vector<double> K = {2 * min_turn_radius_, 0.52, min_turn_radius_};
        // std::cout << "Ki: ";
        // for(auto data : K){
        //     std::cout << data << " ";
        // }
        // std::cout << std::endl;
        // std::vector<double> rules = {n1, n2, n3, 1 / n3, 1 / n2, 1 / n1};
        std::vector<double> rules = {1 / n1, 1 / n2, 1 / n3, n3, n2, n1};
        // std::vector<double> rules = {40, 10, 2, 0.5, 0.05, 0.01};
        // std::vector<double> rules = {40, 40, 10, 1, 0.1, 0.01};
        MyFuzzyController fzctl(subsetNum, inputNum, ruleNum);
        fzctl.setRules(rules);
        costState->VertDist = dl;
        double u = fzctl.realize(inputData, K);
        // std::cout << "scale = " << u;
        // std::cout << "    global_path_len = " << global_path_len_;
        
        // costState->VertCost = distVert * u * des_dir_scale_;
        // costState->VertCost = dMin * u * des_dir_scale_;
        costState->VertCost = dMin * u;
        if(global_path_len_ < min_turn_radius_ && dl < min_turn_radius_ && fabs(global_path_len_ - min_turn_radius_) < eps){
            // costState->VertCost = -costState->VertCost;
            costState->VertCost = 0;
        }
        // if(global_path_len_ > 2 * min_turn_radius_ && dl > 2*min_turn_radius_){
        //     costState->VertCost *= 0.1;
        // }
        // std::cout << "    vertical cost = " << costState->VertCost << std::endl;
        return costState->VertCost;
        // return distVert * fzctl.realize2(inputData, K);

        // IMPROVEMENT VERTICAL COST
        // std::vector<double> inputData = {dl, global_path_len_, abs(abs(tth) - abs(pth)), *min_obs_dist_};
        // std::vector<double> K = {2 * min_turn_radius_, min_turn_radius_, 0.52, 1.0};
        // double p1 = 0.001, p2 = 0.005, p3 = 0.05;
        // std::vector<double> rules = {1 / p1, 1  /p2, 1 / p3, 10, p3, p2, p1};
        // subsetNum = 2;
        // inputNum = 4;
        // ruleNum = 7;
        // MyFuzzyController fzctl(subsetNum, inputNum, ruleNum);
        // fzctl.setRules(rules);
        // costState->VertDist = dl;
        // double u = fzctl.realize3(inputData, K);
        // std::cout << "u = " << u << std::endl;
        // costState->VertCost = dl * u;
        // return costState->VertCost;

    }
}