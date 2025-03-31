#include "DOONLIB/central/chassis.h"

DriveChassis::DriveChassis() 
  : driveMode(doonlib::DRIVE_MODES::TANK_DRIVE)
  , leftDT(nullptr)
  , rightDT(nullptr)
{}

DriveChassis::DriveChassis(
     doonlib::DRIVE_MODES mode, 
     EnhancedMotorGroup* LeftMotors, 
     EnhancedMotorGroup* RightMotors,
     vex::rotation* odomPod_horz,
     vex::rotation* odomPod_vert1, 
     vex::rotation* odomPod_vert2, 
     vex::inertial* imu
) : driveMode(mode)
  , leftDT(LeftMotors)
  , rightDT(RightMotors)
  , horzTracker(odomPod_horz)
  , vertTracker1(odomPod_vert1)
  , vertTracker2(odomPod_vert2)
  , inertial(imu)
{
    leftDT->setMode(doonlib::MOTOR_MODES::DUTY);
    rightDT->setMode(doonlib::MOTOR_MODES::DUTY);
}

void DriveChassis::ArcadeDrive(double leftAxis, double rightAxis) {
    // LeftAxisInput = Controller.Axis3.position(vex::percentUnits::pct);
    // RightAxisInput = Controller.Axis1.position(vex::percentUnits::pct);

    double motor_leftVel = leftAxis + rightAxis;
    double motor_rightVel = leftAxis - rightAxis;

    leftDT->set(motor_leftVel);
    rightDT->set(motor_rightVel);
}

void DriveChassis::TankDrive(double leftAxis, double rightAxis) {
    // LeftAxisInput = Controller.Axis3.position(vex::percentUnits::pct);
    // RightAxisInput = Controller.Axis2.position(vex::percentUnits::pct);

    leftDT->set(leftAxis);
    rightDT->set(rightAxis);
}

void DriveChassis::CurvatureDrive() {} //todo

void DriveChassis::initOdometry() { inertial->calibrate(); }

void DriveChassis::setPose(double x, double y, double heading) { robotPose.setPose(x, y, heading); } 

void DriveChassis::setPodOffsets(
    double leftPodOffset, 
    double rightPodOffset, 
    double strafePodOffset
                                )                               
{
  leftOffset = leftPodOffset;
  rightOffset = rightPodOffset;
  strafeOffset = strafePodOffset;
}

void DriveChassis::configureChassisConstants(
    double gearRatio,
    double rpm,
    double trackWidth,
    double wheelDiameter,
    double verticalPodDiameter, 
    double strafePodDiameter                
                                            )                               
{
    chassis_configs[0] = gearRatio;
    chassis_configs[1] = rpm;
    chassis_configs[2] = trackWidth;
    chassis_configs[3] = wheelDiameter;
    chassis_configs[4] = verticalPodDiameter;
    chassis_configs[5] = strafePodDiameter;
}

void DriveChassis::configureRamseteConstants(
    double kB,
    double kZ, 
    double v_d,
    double w_d
                                            )
{
    ramsete_configs[0] = kB;
    ramsete_configs[1] = kZ;
    ramsete_configs[2] = v_d;
    ramsete_configs[3] = w_d;
}

double DriveChassis::getPose_x() { return robotPose.pose_getX(); }
double DriveChassis::getPose_y() { return robotPose.pose_getY(); }
double DriveChassis::getPose_theta(bool isRadians) { return robotPose.pose_getTheta(isRadians); }

double DriveChassis::track_arcMode() {
    double gearRatio = chassis_configs[0];
    // double trackWidth = chassis_configs[2];
    double verticalTrackingCircumference = chassis_configs[4]*M_PI;
    double horizontalTrackingCircumference = chassis_configs[5]*M_PI;
    double chassisWheelCircumference = chassis_configs[3];

  
    if (horzTracker) {
      // set calculation vars --> strafe pod + vertical imes
        leftDist = (leftDT->getEncoderPosition(false) / 360) * chassisWheelCircumference * gearRatio;
        rightDist = (rightDT->getEncoderPosition(false) / 360) * chassisWheelCircumference * gearRatio;
        strafeDist = (horzTracker->position(vex::degrees) / 360) * horizontalTrackingCircumference; 
    } else if (horzTracker && vertTracker1) { 
        // set calculation vars --> strafe pod + vertical pod + imu prob
        leftDist = (vertTracker1->position(vex::degrees) / 360) * verticalTrackingCircumference;
        rightDist = leftDist;
        strafeDist = (horzTracker->position(vex::degrees) / 360) * horizontalTrackingCircumference;
    } else if (horzTracker && vertTracker1 && vertTracker2) {
        // set calculation vars -->strafe pod + 2 vertical pods + imu
        leftDist = (vertTracker1->position(vex::degrees) / 360) * verticalTrackingCircumference;
        rightDist = (vertTracker2->position(vex::degrees) / 360) * verticalTrackingCircumference;
        strafeDist = (horzTracker->position(vex::degrees) / 360) * horizontalTrackingCircumference;
    } else {
        // user is using full on IME's to run odom
        leftDist = (leftDT->getEncoderPosition(false) / 360) * chassisWheelCircumference * gearRatio;
        rightDist = (rightDT->getEncoderPosition(false) / 360) * chassisWheelCircumference * gearRatio;
        // strafeDist = (horzTracker->position(vex::degrees) / 360) * horizontalTrackingCircumference;
    }

    double deltaLeftDist = leftDist - prevLeftDist;
    double deltaRightDist = rightDist - prevRightDist;
    double deltaStrafeDist = strafeDist - prevStrafeDist;

    prevLeftDist = leftDist;
    prevRightDist = rightDist;
    prevStrafeDist = strafeDist;

    if (inertial) {
        rotation = inertial->rotation() * (M_PI / 180);
    } else { 
        rotation = (leftDist - rightDist) / chassis_configs[2];
    }

    // double wrappedRotation = fmod(rotation + 360, 360); // [0, 360]

    localX = (deltaLeftDist + deltaRightDist) / 2; // delta forward
    localY = deltaStrafeDist; 

    double globalHeading = fmod((rotation * (180 / M_PI)) + 360, 360); // [0, 350]

    double globalX = localX * cos(rotation) - localY * sin(rotation);
    double globalY = localX * sin(rotation) + localY * cos(rotation);
    
    robotPose.updatePose_heading(globalHeading);

    // if the robot only moved sideways then only change pose
    if (std::fabs(localX) < 1e-3+0.0005) {  //0.0015
        robotPose.updatePose_y(deltaStrafeDist);
    } else {
        robotPose.updatePose_x(globalX);
        robotPose.updatePose_y(globalY);
    }

    // if (std::fabs(deltaStrafeDist) > 1e-3+0.0005) { 
    //     robotPose.updatePose_y(globalY); 
    // } else {
    //     robotPose.updatePose_x(globalX);
    //     robotPose.updatePose_y(globalY);
    // }

    return 1.0;
}

double DriveChassis::track_lineMode() { 
    double gearRatio = chassis_configs[0];
    double trackWidth = chassis_configs[2];
    double verticalTrackingCircumference = chassis_configs[4]*M_PI;
    double horizontalTrackingCircumference = chassis_configs[5]*M_PI;
    double chassisWheelCircumference = chassis_configs[3] * M_PI;
    
    if (horzTracker) {
        // set calculation vars --> strafe pod + vertical imes
        leftDist = (leftDT->getEncoderPosition(false) / 360) * chassisWheelCircumference * gearRatio;
        rightDist = (rightDT->getEncoderPosition(false) / 360) * chassisWheelCircumference * gearRatio;
        strafeDist = (horzTracker->position(vex::degrees) / 360) * horizontalTrackingCircumference; 
    } else if (horzTracker && vertTracker1) { 
        // set calculation vars --> strafe pod + vertical pod + imu prob
        leftDist = (vertTracker1->position(vex::degrees) / 360) * verticalTrackingCircumference;
        rightDist = leftDist;
        strafeDist = (horzTracker->position(vex::degrees) / 360) * horizontalTrackingCircumference;
    } else if (horzTracker && vertTracker1 && vertTracker2) {
        // set calculation vars -->strafe pod + 2 vertical pods + imu
        leftDist = (vertTracker1->position(vex::degrees) / 360) * verticalTrackingCircumference;
        rightDist = (vertTracker2->position(vex::degrees) / 360) * verticalTrackingCircumference;
        strafeDist = (horzTracker->position(vex::degrees) / 360) * horizontalTrackingCircumference;
    } else {
        // user is using full on IME's to run odom
        leftDist = (leftDT->getEncoderPosition(false) / 360) * chassisWheelCircumference * gearRatio;
        rightDist = (rightDT->getEncoderPosition(false) / 360) * chassisWheelCircumference * gearRatio;
        strafeDist = (horzTracker->position(vex::degrees) / 360) * horizontalTrackingCircumference;
    }

    double deltaLeftDist = leftDist - prevLeftDist;
    double deltaRightDist = rightDist - prevRightDist;
    double deltaStrafeDist = strafeDist - prevStrafeDist;

    prevLeftDist = leftDist;
    prevRightDist = rightDist;
    prevStrafeDist = strafeDist;

    if (inertial) {
        rotation = inertial->rotation() * (M_PI / 180);
    } else { 
        rotation = (leftDist - rightDist) / trackWidth;
    }

    localX = (deltaLeftDist + deltaRightDist) / 2; // delta forward
    localY = deltaStrafeDist; 

    double globalHeading = fmod((rotation * (180 / M_PI)) + 360, 360);
    
    double globalX = localX * cos(rotation);
    double globalY = localX * sin(rotation);

    robotPose.updatePose_heading(globalHeading);

    if (std::fabs(localX) < 1e-3+0.0005) { 
        // globalY += localY;
        // robotPose.update_y(globalY);
        robotPose.updatePose_y(deltaStrafeDist);
    } else {
        robotPose.updatePose_x(globalX);
        robotPose.updatePose_y(globalY);
    }
  
    return 1.0; 
} 

std::vector<double> DriveChassis::getPose() { return robotPose.getPose(); }


// needs to be motion profiled
double DriveChassis::RamseteControl(double x, double y, double heading) { 

    /*target_error_x = x_desired - x_actual
    target_error_y = y_desired - y_actual
    "--> means its irrelevant
    target_error_theta = theta_desired - theta_actual
    e_x = cos(theta) * target_error_x + sin(theta) * target_error_y + --> 0 * target_error_theta
    e_y = -sin(theta) * target_error_x + cos(theta) * target_error_y + --> 0 * target_error_theta
    e_theta = --> 0 * target_error_x + --> 0 * target_error_y + 1 * target_error_theta*/
    double trackWidth = chassis_configs[2];
    double wheelRadius = chassis_configs[3] / 2;

    double k_b = ramsete_configs[0];
    double k_z = ramsete_configs[1];
    double v_d = ramsete_configs[2];
    double w_d = ramsete_configs[3];

    double desired_x = x;
    double desired_y = y;
    double desired_heading = heading;

    double tarX_e = desired_x - robotPose.pose_getX();
    double tarY_e = desired_y - robotPose.pose_getY();
    double tarTheta_e = desired_heading - robotPose.pose_getTheta(true);

    tarTheta_e = fmod(tarTheta_e + M_PI, 2 * M_PI) - M_PI; // normalize domain to [-pi, pi]

    double e_x = cos(robotPose.pose_getTheta(true)) * tarX_e + sin(robotPose.pose_getTheta(true)) * tarY_e;
    double e_y = -sin(robotPose.pose_getTheta(true)) * tarX_e + cos(robotPose.pose_getTheta(true)) * tarY_e;
    double e_theta = tarTheta_e;

    double k = 2 * k_z * sqrt(pow(w_d, 2) + k_b * pow(v_d, 2)); // controller gain

    /*
        Desired velocities are best when they come from a pre-computed path 
        however having the vels at 0 when required will give no controller output
        Solution: create small scalar values for the velocities to take
    */

    double v = v_d * cos(e_theta) + k * e_x;
    double w = w_d + k * e_theta + (k_b * v_d * sin(e_theta) * e_y) / e_theta;

    double leftMotorVelocity = (v - (w * trackWidth / 2)) / wheelRadius;
    double rightMotorVelocity = (v + (w * trackWidth / 2)) / wheelRadius;

    leftDT->set(leftMotorVelocity);
    rightDT->set(rightMotorVelocity);

    return 1.0;

}

