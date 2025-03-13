#pragma once

#include "vex.h"
#include "DOONLIB/motion-utils.hpp"
#include "DOONLIB/vex-wrappers/EnhancedMotors.h"

class DriveChassis {

    private:

        doonlib::DRIVE_MODES driveMode;

        vex::controller& Controller;

        EnhancedMotorGroup& leftDT;
        EnhancedMotorGroup& rightDT;

        /* Odometry Configurations */
        vex::rotation* horizontalTracker;
        vex::rotation* verticalTracker;
        // vex::rotation& verticalTracker;

        double LeftAxisInput;
        double RightAxisInput;

        enum ODOM_DRIVE_MODES {
            RAMSETE,
            BOOMERANG,
            RAMPID,
            PID
        };

    protected:

        /* Drive Types */
        void ArcadeDrive();

        void TankDrive();

        void CurvatureDrive();

        /* Odometry */
        void calculateOdom_line(void); // line approximation
        void calculateOdom_arc(void); // arc approximation


    public:

        DriveChassis(doonlib::DRIVE_MODES mode, vex::controller& master,
                     EnhancedMotorGroup& LeftMotors, EnhancedMotorGroup& RightMotors);

        /* Command Drive */
        void controlChassis(double LeftAxis, double RightAxis);

        /* Odometry */
        void enableOdom(bool enable);

};