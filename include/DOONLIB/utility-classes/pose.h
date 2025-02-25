#pragma once

#include <vector>
#include "vex.h"

class Pose {

    private:

        double x, y, theta;
        std::vector<double> curPose = {x, y, theta};
        
    public:

        Pose(std::string poseName);

        double setPose(double x, double y, double theta);
        double getPose();

        double pose_getX();
        double pose_getY();
        double pose_getTheta(bool isRadians);

};