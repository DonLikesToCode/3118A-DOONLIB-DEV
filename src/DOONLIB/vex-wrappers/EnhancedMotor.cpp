#include "DOONLIB/central/doon_main.h"

EnhancedMotor::EnhancedMotor(int port_num, vex::gearSetting gearType)
    : vex::motor(port_num, gearType, isForward)
{ 
    isForward = (port_num > 0);
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

void EnhancedMotor::writeAll(void) { 
    if (controlMode == DUTY) {
        this->spin(vex::directionType::fwd, motor_cmd, vex::voltageUnits::volt); 
    } else if (controlMode == POSITION) {
        //pos logic todo
    } else if (controlMode == VELOCITY) { 
        //velocity logic todo
    }
}