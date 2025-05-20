#pragma once
#include "DOONLIB/central/doon_api.h"

using namespace vex;

extern brain Brain;
extern controller Controller1;

// extern EnhancedMotor test_motor;

extern EnhancedMotor LMotorA;
extern EnhancedMotor LMotorB;
extern EnhancedMotor LMotorC;

extern EnhancedMotor RMotorA;
extern EnhancedMotor RMotorB;
extern EnhancedMotor RMotorC;

extern EnhancedMotorGroup LMotorGroup;
extern EnhancedMotorGroup RMotorGroup;

void vexcodeInit(void);