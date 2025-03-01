#include "DOONLIB/central/doon_api.h"
#include "DOONLIB/central/chassis.h"

DriveChassis::DriveChassis(doonlib::DRIVE_MODES mode, vex::controller& master,
                           vex::motor_group& LeftMotors, vex::motor_group& RightMotors)
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

    leftDT.spin(vex::directionType::fwd, motor_leftVel, vex::percentUnits::pct);
    rightDT.spin(vex::directionType::fwd, motor_rightVel, vex::percentUnits::pct);
}

void DriveChassis::TankDrive() {
    LeftAxisInput = Controller.Axis3.position(vex::percentUnits::pct);
    RightAxisInput = Controller.Axis2.position(vex::percentUnits::pct);

    leftDT.spin(vex::directionType::fwd, LeftAxisInput, vex::percentUnits::pct);
    rightDT.spin(vex::directionType::fwd, RightAxisInput, vex::percentUnits::pct);
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