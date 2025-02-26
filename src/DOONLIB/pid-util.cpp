#include "DOONLIB/central/doon_api.h"

PID::PID(double cmd, double tolerance, bool slew, double dt)
    : controlled_cmd(cmd)
    , settleTolerance(tolerance)
    , kP(1)
    , kI(0)
    , kD(0)
    , given_dt(dt)
{}
void PID::setConstants(double sentP, double sentI, double sentD) {
    kP = sentP; 
    kI = sentI; 
    kD = sentD;
}

void PID::setIntegralMax(double given) { maxIntegral = given; }

void PID::resetCalculationValues() {
    integral = 0;
    derivative = 0;
    error = 0;
    preverror = 0;
    controlled_cmd = 0;
}

bool PID::isComplete() { return fabs(error) < settleTolerance; }

double PID::getP() { return kP; }
double PID::getI() { return kI; }
double PID::getD() { return kD; }
double PID::getError() { return error; }

double PID::calculate(double cmd) {
    static Timer dt_pid("delta-time-pid");
    dt_pid.timer_setDeltaRate(given_dt);
    dt_pid.timer_startDelta();

    error = cmd; //kP

    double dt = dt_pid.timer_getDelta();
    if (dt < 1e-6) { dt = 1e-6; } //0 division protection

    // if (doon_utils::sign(error) != doon_utils::sign(preverror)) { 
    //     integral = 0;
    //     integral += error * dt; 
    // }

    // if (fabs(integral + error * dt) < maxIntegral) { 
    //     integral += error * dt; 
    // } else { 
    //     integral = maxIntegral * doon_utils::sign(integral); 
    // }
    
    integral += error * dt;
    integral = doon_utils::clamp(integral, -maxIntegral, maxIntegral);

    derivative = (error-preverror) / dt; //kD

    double cmd_pidOutput = (kP * error) + (kI * integral) + (kD * derivative);
    preverror = error;

    controlled_cmd = cmd_pidOutput;

    dt_pid.timer_endDelta();

    return controlled_cmd;
}