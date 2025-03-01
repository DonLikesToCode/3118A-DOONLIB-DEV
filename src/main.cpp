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
#include "DOONLIB/central/doon_api.h"
#include "devices.hpp"

using namespace vex;

competition Competition;

void pre_auton(void) { vexcodeInit(); }

void Intialize() {

  

}

void HandleInput() {

  

}

void UpdateRobot() { 

  /* Anything that needs to be constantly updated goes here (i.e input, writing command to motors) */
  /* Please do also ensure that your subsystem periodic functions run here */

}

void autonomous(void) {
  
}

void usercontrol(void) {
  while (true) {
    HandleInput();
    UpdateRobot();

    wait(15, msec);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
