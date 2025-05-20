/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "robot-config.h"
#include "DOONLIB/central/doon_api.h"

using namespace vex;

competition Competition;

void pre_auton(void) { vexcodeInit(); }

/*

*/

DriveChassis Chassis(
                    &LMotorGroup, 
                    &RMotorGroup,
                    nullptr,
                    nullptr,
                    nullptr
);



void Intialize() {

  /* Initialize any robot configurations required before teleop here */

}

void HandleInput() {

  /* Map your subsystem inputs/buttons here */

}

void Periodic() { 

  /* 
   * Anything that needs to be constantly updated needs to go under here.
   * You can also probably map your inputs here, but it's good practice to 
   * have your inputs separated from your other robot functions (i.e odom).
  */
  
}

void autonomous(void) {
  
}

void usercontrol(void) {
  while (true) {
    HandleInput();
    Periodic();

    wait(15, msec);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  Intialize();

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
