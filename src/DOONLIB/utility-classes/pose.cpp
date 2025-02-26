#pragma once

#include "DOONLIB/utility-classes/pose.h"

Pose::Pose(std::string poseName) 
    : x(0)
    , y(0)
    , theta(0) 
{}

double Pose::setPose(double x, double y, double theta) {
    curPose[0] = x;
    curPose[1] = y;
    curPose[2] = theta;
}

double Pose::getPose() { return curPose; }

double Pose::pose_getX() { return curPose[0]; }
double Pose::pose_getY() { return curPose[1]; }
double Pose::pose_getTheta(bool isRadians) { isRadians ? return curPose[2] : return curPose[2] * M_PI/180; }