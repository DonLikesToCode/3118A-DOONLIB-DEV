#pragma once

#include <vector>
#include "vex.h"

class Pose {

    private:

        double x, y, theta;
        std::vector<double> curPose = {x, y, theta};
        
    public:

        Pose(std::string poseName);

        void setPose(double x, double y, double theta);
        std::vector<double> getPose();

        double pose_getX();
        double pose_getY();
        double pose_getTheta(bool isRadians);

};