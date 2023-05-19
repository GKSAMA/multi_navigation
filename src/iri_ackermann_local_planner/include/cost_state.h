#ifndef COST_STATE_H
#define COST_STATE_H

#include <iostream>

struct CostState {
    double DirDiff;
    double DirCost;
    double VertDist;
    double VertCost;
    double Traj_x;
    double Traj_y;
    void init()
    {
        DirDiff = 0;
        DirCost = 0;
        VertDist = 0;
        VertCost = 0;
        Traj_x = 0;
        Traj_y = 0;
    }
};

#endif