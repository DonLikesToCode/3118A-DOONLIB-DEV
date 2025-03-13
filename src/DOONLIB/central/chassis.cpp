#include "DOONLIB/central/chassis.h"

DriveChassis::DriveChassis(doonlib::DRIVE_MODES mode, vex::controller& master,
                           EnhancedMotorGroup& LeftMotors, EnhancedMotorGroup& RightMotors)
    : driveMode(mode)
    , Controller(master)
    , leftDT(LeftMotors)
    , rightDT(RightMotors)
{}

void DriveChassis::ArcadeDrive() {
    LeftAxisInput = Controller.Axis3.position(vex::percentUnits::pct);
    RightAxisInput = Controller.Axis1.position(vex::percentUnits::pct);

    double motor_leftVel = LeftAxisInput + RightAxisInput;
    double motor_rightVel = LeftAxisInput - RightAxisInput;

    leftDT.writeCommand(motor_leftVel);
    rightDT.writeCommand(motor_rightVel);
}

void DriveChassis::TankDrive() {
    LeftAxisInput = Controller.Axis3.position(vex::percentUnits::pct);
    RightAxisInput = Controller.Axis2.position(vex::percentUnits::pct);

    leftDT.writeCommand(LeftAxisInput);
    rightDT.writeCommand(RightAxisInput);
}

void DriveChassis::CurvatureDrive() {
    // todo
}

void DriveChassis::controlChassis(double LeftAxis, double RightAxis) {
    if (driveMode == doonlib::DRIVE_MODES::ARCADE_DRIVE) {
        ArcadeDrive();
    } else if (driveMode == doonlib::DRIVE_MODES::TANK_DRIVE) {
        TankDrive();
    } else if (driveMode == doonlib::DRIVE_MODES::CURVATURE_DRIVE) {
        CurvatureDrive();
    }
}