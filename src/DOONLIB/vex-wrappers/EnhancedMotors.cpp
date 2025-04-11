#include "DOONLIB/vex-wrappers/EnhancedMotors.h"
/* Vex Motor Wrapper */

EnhancedMotor::EnhancedMotor(int port_num, vex::gearSetting gearType, doonlib::MOTOR_MODES motor_mode)
    : vex::motor(port_num, gearType, isReverse)
    , controlMode(motor_mode)
    , motor_cmd(0)
    , isReverse(port_num < 0)
    , motorPID(motor_cmd, 1.5, false, 0.01)
{ 
    // isForward = (port_num > 0);
    motorPID.setConstants(mtr_kP, mtr_kI, mtr_kD);
}

void EnhancedMotor::setMode(doonlib::MOTOR_MODES given) { controlMode = given; }

// void EnhancedMotor::setCommand(double cmd) { motor_cmd = cmd; }
double EnhancedMotor::getCommand() { return motor_cmd; }

double EnhancedMotor::getEncoderPosition(bool isRadians) { 
    if (isRadians) { return doon_utils::degToRad(this->position(vex::degrees)); } 
    else { return this->position(vex::degrees); }
}

void EnhancedMotor::setEncoderPosition(double given) {
    this->setPosition(given, vex::degrees);
}

void EnhancedMotor::resetEncoderPosition() {
    this->resetPosition();
}

void EnhancedMotor::set(double cmd) {
    motor_cmd = cmd;
    if (controlMode == doonlib::MOTOR_MODES::DUTY) {
        //motor_cmd is treated as the percentage or literal cmd to spin the motors at.
        this->spin(vex::directionType::fwd, motor_cmd, vex::voltageUnits::volt); 
    } else if (controlMode == doonlib::MOTOR_MODES::POSITION) {
        //motor_cmd is treated as the target position
        double targetPosition = motor_cmd;
        double currentPosition = this->position(vex::degrees);
        double pidOutput = motorPID.calculate(targetPosition-currentPosition);

        if (fabs(targetPosition-currentPosition) < mtr_tolerance) { motorPID.resetCalculationValues(); }

        pidOutput = doon_utils::clamp(pidOutput, -12.0, 12.0);
        this->spin(vex::directionType::fwd, pidOutput, vex::voltageUnits::volt);
    } else if (controlMode == doonlib::MOTOR_MODES::VELOCITY) { 
        //motor_cmd is treated as the target velocity in rpm
        double targetVelocity = motor_cmd;
        double currentVelocity = this->velocity(vex::velocityUnits::rpm);
        double pidOutput = motorPID.calculate(targetVelocity-currentVelocity);

        if (fabs(targetVelocity-currentVelocity) < 1e-3) { motorPID.resetCalculationValues(); }

        pidOutput = doon_utils::clamp(pidOutput, -12.0, 12.0);
        this->spin(vex::directionType::fwd, pidOutput, vex::voltageUnits::volt);
    }
}

// void EnhancedMotor::setP(double kP) { mtr_kP = kP; }
// void EnhancedMotor::setI(double kI) { mtr_kI = kI; }
// void EnhancedMotor::setD(double kD) { mtr_kD = kD; }

void EnhancedMotor::setMotorPIDConstants(double kP, double kI, double kD) {
    mtr_kP = kP;
    mtr_kI = kI;
    mtr_kD = kD;

    motorPID.setConstants(mtr_kP, mtr_kI, mtr_kD);
}

void EnhancedMotor::setTolerance(double kTol) { mtr_tolerance = kTol; }

/* Vex Motorgroup Wrapper */

EnhancedMotorGroup::EnhancedMotorGroup(std::vector<EnhancedMotor*> motors) 
    : currentMotors(motors)
    , motor_cmd(0)
    , motorPID(motor_cmd, 1.5, false, 0.01)
{}

void EnhancedMotorGroup::setMode(doonlib::MOTOR_MODES given) { controlMode = given; }

double EnhancedMotorGroup::getEncoderPosition(bool isRadians) {
    double total = 0.0;
    int count = 0;

    for (auto& motor : currentMotors) { 
        if (motor) { 
            total += isRadians ? doon_utils::degToRad(motor->position(vex::degrees)) 
                               : motor->position(vex::degrees);
            count++;
        }
    }
    
    return (count > 0) ? (total / count) : 0.0; 
}

void EnhancedMotorGroup::setEncoderPosition(double given) {
    for (auto& motor : currentMotors) { 
        if (motor) { motor->setPosition(given, vex::degrees); }
    }
}

void EnhancedMotorGroup::resetEncoderPosition() {
    for (auto& motor : currentMotors) {
        if (motor) { motor->resetEncoderPosition(); }
    }
}

// void EnhancedMotorGroup::setCommand(double cmd) { motor_cmd = cmd; }

double EnhancedMotorGroup::getCommand() { return motor_cmd; }

void EnhancedMotorGroup::set(double cmd) { 
    motor_cmd = cmd;
    for (auto& motor : currentMotors) {
        if (motor) {
            if (controlMode == doonlib::MOTOR_MODES::DUTY) {
                //motor_cmd is treated as the percentage or literal cmd to spin the motors at.
                motor->spin(vex::directionType::fwd, motor_cmd, vex::voltageUnits::volt); 
            } else if (controlMode == doonlib::MOTOR_MODES::POSITION) {
                //motor_cmd is treated as the target position
                double targetPosition = motor_cmd;
                double currentPosition = motor->position(vex::degrees);
                double pidOutput = motorPID.calculate(targetPosition-currentPosition);
        
                if (fabs(targetPosition-currentPosition) < mtr_tolerance) { motorPID.resetCalculationValues(); }
        
                pidOutput = doon_utils::clamp(pidOutput, -12.0, 12.0);
                motor->spin(vex::directionType::fwd, pidOutput, vex::voltageUnits::volt);
            } else if (controlMode == doonlib::MOTOR_MODES::VELOCITY) { 
                //motor_cmd is treated as the target velocity in rpm
                double targetVelocity = motor_cmd;
                double currentVelocity = motor->velocity(vex::velocityUnits::rpm);
                double pidOutput = motorPID.calculate(targetVelocity-currentVelocity);
        
                if (fabs(targetVelocity-currentVelocity) < mtr_tolerance) { motorPID.resetCalculationValues(); }
        
                pidOutput = doon_utils::clamp(pidOutput, -12.0, 12.0);
                motor->spin(vex::directionType::fwd, pidOutput, vex::voltageUnits::volt);
            }
        }

    }
}

// void EnhancedMotorGroup::setP(double kP) { 
//     for (auto& motor : currentMotors) {
//         if (motor) { motor->setP(kP); }
//     }
// }

// void EnhancedMotorGroup::setI(double kI) { 
//     for (auto& motor : currentMotors) {
//         if (motor) { motor->setI(kI); }
//     }
// }

// void EnhancedMotorGroup::setD(double kD) { 
//     for (auto& motor : currentMotors) {
//         if (motor) { motor->setD(kD); }
//     }
// }

void EnhancedMotorGroup::setGroupPIDConstants(double kP, double kI, double kD) {
    for (auto&motor : currentMotors) {
        if (motor) { motor->setMotorPIDConstants(kP, kI, kD); }
    }
}

void EnhancedMotorGroup::setTolerance(double kTol) {
    for (auto&motor : currentMotors) { 
        if (motor) { motor->setTolerance(kTol); }
    }
}