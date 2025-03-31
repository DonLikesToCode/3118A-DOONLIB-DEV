#include "DOONLIB/utility-classes/pose.h"

Pose::Pose()
    : x(0)
    , y(0)
    , theta(0)
{}

void Pose::setPose(double x, double y, double theta) {
    curPose[0] = x;
    curPose[1] = y;
    curPose[2] = theta;
}

std::vector<double> Pose::getPose() { return curPose; }

double Pose::pose_getX() { return curPose[0]; }
double Pose::pose_getY() { return curPose[1]; }
double Pose::pose_getTheta(bool isRadians) { return isRadians ? curPose[2] : curPose[2] * M_PI / 180; }

void Pose::updatePose_x(double addX) { curPose[0] += addX; }
void Pose::updatePose_y(double addY) { curPose[1] += addY; }
void Pose::updatePose_heading(double addDeg) { (curPose[2] * M_PI / 180) + addDeg; }

double Pose::getDistTo(Pose& cPose) {
    //dist formula = sqrt( (x_2-x_1)^2 + (y_2-y_1)^2 )

    double delta_x = cPose.pose_getX() - this->pose_getX();
    double delta_y = cPose.pose_getY() - this->pose_getY();

    return sqrt(pow(delta_x, 2) + pow(delta_y, 2));
}