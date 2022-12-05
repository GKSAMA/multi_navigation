#include "adwa_local_planner/min_turn_radius_cost.h"
#include "angles/angles.h"

namespace base_local_planner{
    MinTurnRadiusCostFunction::MinTurnRadiusCostFunction(){

    }

    bool MinTurnRadiusCostFunction::prepare(){
        return true;
    }

    void MinTurnRadiusCostFunction::setScale(double scale) {
        this->turnCostScale = scale;
    }

    void MinTurnRadiusCostFunction::setAngularVelocity(double cur_angular_velocity) {
        this->curAngularVelocity = cur_angular_velocity;
    }

    void MinTurnRadiusCostFunction::setFixedParams(double max_angular_velocity,
                                                   double wheel_base){
        this->maxAngularVelocity = max_angular_velocity;
        this->wheelbase = wheel_base;
        this->minTurnRadius =  this->wheelbase / tan(this->maxAngularVelocity);
    }

    double MinTurnRadiusCostFunction::scoreTrajectory(Trajectory &traj){
        double threshold = 1e-4;
        double cost = 0.f;
        double xf, yf, yaw;
        double xcenter,ycenter;
        double xr, yr;
        traj.getPoint(0, xf, yf, yaw);
        yaw = angles::normalize_angle(yaw);
        // line1 is robot orientation
        double k1 = tan(yaw);
        double b1;
        double k2, b2;
        double xo, yo;
        // compute rear position and line2 which perpendicular to robot orientation.
        if(abs(yaw) < threshold || M_PI - abs(yaw) < threshold) { // robot orientation is horizontal
            if(abs(yaw) < threshold) { // judge robot ahead left or right
                xr = xf - wheelbase;
            } else {
                xr = xf + wheelbase;
            }
            yr = yf;
            k2 = INT_MAX;
            b2 = INT_MAX;
        } else if((abs(yaw) - M_PI / 2) < threshold) { // robot orientation is vertical
            if(yaw > 0) {   // judge robot ahead up or down
                yr = yf - wheelbase;
            } else {
                yr = yf + wheelbase;
            }
            xr = xf;
            k2 = 0;
            b2 = yr;
        }else if(yaw > 0) { // robot ahead up (front wheel y position larger than rear wheel)
            if(k1 > 0) { // robot ahead left or right
                xr = -wheelbase / sqrt(k1 * k1 + 1) + xf;
            } else {
                xr = wheelbase / sqrt(k1 * k1 + 1) + xf;
            }
            yr = (xr - xf) * k1 + yf;
            k2 = -1 / k1;
            b2 = xr / k1 + yr;
        } else { // robot ahead down (front wheel y position less than rear wheel)
            if(k1 > 0) { // robot ahead left or right
                xr = wheelbase / sqrt(k1 * k1 + 1) + xf;
            } else {
                xr = -wheelbase / sqrt(k1 * k1 + 1) + xf;
            }
            yr = (xr - xf) * k1 + yf;
            k2 = -1 / k1;
            b2 = xr / k1 + yr;
        }
        // compute line3 which perpendicular to front wheel orientation
        double k3, b3;
        if(abs(yaw + curAngularVelocity) < threshold || M_PI - abs(yaw + curAngularVelocity) < threshold) {
            k3 = INT_MAX;
            b3 = INT_MAX;
        } else if(M_PI / 2 - abs(yaw + curAngularVelocity) < threshold) {
            k3 = 0;
            b3 = yf;
        } else {
            k3 = -1 / tan(yaw + curAngularVelocity);
            b3 = -k3 * xf + yf;
        }

        // compute the turn center.
        bool forward = false;
        if(k2 == 0) { // line2 is parallel to x axis.
            if(abs(curAngularVelocity) > threshold) { // robot not forward
                yo = yr;
                xo = (yf - yo) / (-k3) + xf;
            } else {
                yo = INT_MAX;
                xo = INT_MAX;
                forward = true;
            }
        } else if(k2 == INT_MAX){ // line2 is perpendicular to x axis
            if(abs(curAngularVelocity) > threshold) { // robot not forward
                yo = yr + minTurnRadius;
                xo = (yf - yo) / (-k3) + xf;
            } else {
                yo = INT_MAX;
                xo = INT_MAX;
                forward = true;
            }
        } else {
            // front wheel's orientation is parallel to x axis
            if(abs(curAngularVelocity + yaw) < threshold || M_PI - abs(curAngularVelocity + yaw) < threshold){
                xo = xf;
                yo = k2 * (xo - xr) + yr;
            } else if(M_PI / 2 - abs(curAngularVelocity + yaw) < threshold){ // front wheel's orientation is perpendicular to x axis 
                yo = yf;
                xo = (yr - yo) * k1;
            } else {
                xo = (b2 - b3) / (k3 - k2);
                yo = k2 * (xo - xr) + yr;
            }
        }

        // if robot move forward, there is no need to compute turn center.
        if(forward) {
            cost = 1.f;
        } else {
            // compute distance between turn center and each point in trajectory.
            // every points must out of the minmal turn circle.
            double px, py, pth;
            bool legal = true;
            for(unsigned int i = 0; i < traj.getPointsSize(); ++i){
                traj.getPoint(i, px, py, pth);
                double distSqr = (px - xo) * (px - xo) + (py - yo) * (py - yo);
                if(distSqr < minTurnRadius) {
                    legal = false;
                    break;
                }
            }
            if(legal){
                cost = 1.f;
            } else {
                cost = -1.f;
            }
        }
        return cost * turnCostScale;
    }
}