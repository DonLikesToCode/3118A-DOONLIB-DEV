#pragma once

#include "vex.h"
#include "DOONLIB/motion-utils.hpp"

class DriveChassis {

    private:

        doonlib::DRIVE_MODES driveMode;

        vex::controller& Controller;

        vex::motor_group& leftDT;
        vex::motor_group& rightDT;

        double LeftAxisInput;
        double RightAxisInput;

    protected:

        /* Drive Types */
        void ArcadeDrive();

        void TankDrive();

        void CurvatureDrive();

    public:

        DriveChassis(doonlib::DRIVE_MODES mode, vex::controller& master,
                     vex::motor_group& LeftMotors, vex::motor_group& RightMotors);

        /* Command Drive */
        void controlChassis(double LeftAxis, double RightAxis);

};