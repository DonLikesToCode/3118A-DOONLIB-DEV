#include "DOONLIB/pid-util.h"

PID::PID(double cmd, double tolerance, bool slew, double dt)
    : controlled_cmd(cmd)
    , settleTolerance(tolerance)
    , kP(1)
    , kI(0)
    , kD(0)
    , given_dt(dt)
{
    dt_pid.timer_startDelta();
    dt_pid.timer_setDeltaRate(dt);

    if (tolerance <= 0) { tolerance = 2; }
}

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
    dt_pid.timer_endDelta();
    double dt = dt_pid.timer_getDelta();
    dt_pid.timer_startDelta();

    error = cmd;

    if (dt < 1e-2) { dt = 1e-2; } // division by 0 protection
    
    integral += error * dt;
    integral = doon_utils::clamp(integral, -maxIntegral, maxIntegral);

    derivative = (error-preverror) / dt; 

    double cmd_pidOutput = (kP * error) + (kI * integral) + (kD * derivative);
    preverror = error;

    controlled_cmd = cmd_pidOutput;

    dt_pid.timer_endDelta();

    return controlled_cmd;
}