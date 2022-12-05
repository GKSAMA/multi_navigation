#include "adwa_local_planner/vertical_distance_cost.h"
#include <fuzzy_controller/my_fuzzy_controller.h>
#include "angles/angles.h"
#include <math.h>
#include <iostream>
#include <queue>
#include <vector>
#include <climits>

#define MAP_INDEX(map, i, j) ((i) + (j) * map.size_x)
#define MAP_WXGX(map, i) (map.origin_x + (i - map.size_x / 2) * map.scale)
#define MAP_WYGY(map, i) (map.origin_y + (i - map.size_y / 2) * map.scale)

namespace base_local_planner{
    VerticalDistanceCostFunction::VerticalDistanceCostFunction(costmap_2d::Costmap2D* costmap)
        : costmap_(costmap)
    {
        if(costmap != NULL){
            world_model_ = new CostmapModel(*costmap_);
        }
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

    void VerticalDistanceCostFunction::SetTargetPose(geometry_msgs::PoseStamped target){
        target_pose = target;
    }

    void VerticalDistanceCostFunction::SetScale(double scale) {
        distance_scale = scale;
    }

    void VerticalDistanceCostFunction::ComputeTurnRadius(double wheelbase, double max_steer_angle)
    {
        min_turn_radius_ = sqrt(wheelbase * wheelbase / 4 + wheelbase * wheelbase / (cos(max_steer_angle) * cos(max_steer_angle)));
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

        // cout << "map size: width : " << map_info.size_x << "  height : " << map_info.size_y << std::endl;
        // std::cout << "origin x : " << map_info.origin_x << " origin y : " << map_info.origin_y << std::endl;
        // std::cout << " resolution : " << map_info.scale << std::endl;
        // min_obstacle_dist_ = INT_MAX;
        std::vector<bool> visited(map_info.size_x * map_info.size_y, false);
        std::vector<std::pair<int,int>> dir = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
        std::queue<std::pair<int,int>> qu;
        qu.push(std::make_pair((x - map_info.origin_x) / map_info.scale, (y - map_info.origin_y) / map_info.scale));
        visited[MAP_INDEX(map_info, x, y)] = true;
        // int count  = 10000;
        while (!qu.empty()){
            int levelNum = qu.size();
            for(int i = 0; i < levelNum; ++i){
                int curx = qu.front().first;
                int cury = qu.front().second;
                // std::cout << "cur x : " << curx << " cur y : " << cury << std::endl;
                qu.pop();
                if(costmap_->getCost(curx, cury) >= 240){
                    // return std::sqrt(pow((x - curx), 2) + pow((y - cury), 2));
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
        double eps = 1e-4;
        double px, py, pth, fx, fy, fth;
        int finalIndex = traj.getPointsSize() - 1;
        traj.getPoint(finalIndex, px, py, pth);
        fth = pth;
        fx = px;
        fy = py;
        double tx, ty, tth;
        tx = target_pose.pose.position.x; 
        ty = target_pose.pose.position.y;
        tf2::Transform pose_tf;
        tf2::convert(target_pose.pose,pose_tf);
        double useless_pitch, useless_roll;
        pose_tf.getBasis().getEulerYPR(tth, useless_pitch, useless_roll);
        tth = angles::normalize_angle(tth);
        tth = tf2::getYaw(target_pose.pose.orientation);
        double nx, ny;
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
        for(unsigned int i = 0; i < traj.getPointsSize(); ++i){
            traj.getPoint(i, px, py, pth);
            double curDistance = std::abs(k * px - py + b) / std::sqrt(k * k + 1 * 1);
            dMax = std::max(dMax, curDistance);
        }
        double distVert = 1 - (dl / dMax);
        int subsetNum = 2;
        int inputNum = 3;
        int ruleNum = 6;
        double n1 = 0.01, n2 = 0.05, n3 = 0.5;
        double targetMinObsDist = ComputeMinObstacleDistance(fx,fy);
        // std::cout << "targetMinObsDist : " << targetMinObsDist << std::endl;
        std::vector<double> inputData = {dl, abs(abs(tth) - abs(fth)), targetMinObsDist};
        // std::cout << "inputData: ";
        // for(auto data : inputData){
        //     std::cout << data << " ";
        // }
        // std::cout << std::endl;
        std::vector<double> K = {min_turn_radius_, 0.52, 0.5};
        std::vector<double> rules = {n1, n2, n3, 1 / n3, 1 / n2, 1 / n1};
        MyFuzzyController fzctl(subsetNum, inputNum, ruleNum);
        fzctl.setRules(rules);

        return distVert * fzctl.realize(inputData, K);
    }
}