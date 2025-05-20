#include "vex.h"
#include "DOONLIB/central/doon_api.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
controller Controller1 = controller(primary);

// EnhancedMotor test_motor(1, ratio18_1);

EnhancedMotor LMotorA(1, ratio18_1, doonlib::MOTOR_MODES::DUTY);
EnhancedMotor LMotorB(2, ratio18_1, doonlib::MOTOR_MODES::DUTY);
EnhancedMotor LMotorC(3, ratio18_1, doonlib::MOTOR_MODES::DUTY);

EnhancedMotor RMotorA(11, ratio18_1, doonlib::MOTOR_MODES::DUTY);
EnhancedMotor RMotorB(12, ratio18_1, doonlib::MOTOR_MODES::DUTY);
EnhancedMotor RMotorC(13, ratio18_1, doonlib::MOTOR_MODES::DUTY);

EnhancedMotorGroup LMotorGroup({&LMotorA, &LMotorB, &LMotorC});
EnhancedMotorGroup RMotorGroup({&RMotorA, &RMotorB, &RMotorC});

void vexcodeInit(void) {}