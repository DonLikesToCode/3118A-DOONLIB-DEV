#include "DOONLIB/vex-wrappers/EnhancedMotor.h"

EnhancedMotor::EnhancedMotor(int port_num, vex::gearSetting gearType)
    : vex::motor(port_num, gearType, isForward)
    , motor_cmd(0)
    , motorPID(motor_cmd, 1.5, false, 0.01)
{ 
    isForward = (port_num > 0);
    motorPID.setConstants(1, 0, 0);
}

void EnhancedMotor::setMode(MOTOR_MODE given) { controlMode = given; }

void EnhancedMotor::setCmd(double cmd) { motor_cmd = cmd; }

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

void EnhancedMotor::writeCommands(void) { 
    if (controlMode == DUTY) {
        //motor_cmd is treated as the percentage or literal cmd to spin the motors at.
        this->spin(vex::directionType::fwd, motor_cmd, vex::voltageUnits::volt); 
    } else if (controlMode == POSITION) {
        //motor_cmd is treated as the target position
        double targetPosition = motor_cmd;
        double currentPosition = this->position(vex::degrees);
        double pidOutput = motorPID.calculate(targetPosition-currentPosition);

        if (fabs(targetPosition-currentPosition) < 2.0) { motorPID.resetCalculationValues(); }

        pidOutput = doon_utils::clamp(pidOutput, -12.0, 12.0);
        this->spin(vex::directionType::fwd, pidOutput, vex::voltageUnits::volt);
    } else if (controlMode == VELOCITY) { 
        //motor_cmd is treated as the target velocity in rpm
        double targetVelocity = motor_cmd;
        double currentVelocity = this->velocity(vex::velocityUnits::rpm);
        double pidOutput = motorPID.calculate(targetVelocity-currentVelocity);

        if (fabs(targetVelocity-currentVelocity) < 2.0) { motorPID.resetCalculationValues(); }

        pidOutput = doon_utils::clamp(pidOutput, -12.0, 12.0);
        this->spin(vex::directionType::fwd, pidOutput, vex::voltageUnits::volt);
    }
}