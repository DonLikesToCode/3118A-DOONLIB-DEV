#pragma once

#include <vector>

#include "vex.h"

#include "DOONLIB/central/doon_api.h"


class Pose {

    private:

        double x, y, theta;
        
    public:

        Pose();

        std::vector<double> curPose = {x, y, theta};

        void setPose(double x, double y, double theta);
        std::vector<double> getPose();

        double pose_getX();
        double pose_getY();
        double pose_getTheta(bool isRadians); 

        void updatePose_x(double addX); // updates through adding val
        void updatePose_y(double addY); // updates through adding val
        void updatePose_heading(double addDeg); // updates through adding val

        double getDistTo(Pose& pose); // only uses x and y to get dist

};