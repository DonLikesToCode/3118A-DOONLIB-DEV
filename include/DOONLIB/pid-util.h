#pragma once

#include "vex.h"

class PID {

    private:

        /* Defaults */
        double controlled_cmd;
        double settleTolerance, settleTime;

        double kP, kI, kD;
        double integral, maxIntegral{0.05*kP}, derivative;

        double given_dt;

        double error, preverror;

        /* Slew Rate*/
        bool isSlew;

    public:

        PID(double cmd, double tolerance, bool slew = false, double dt = 10);

        void pid_setConstants(double sent_kp, double sent_ki, double send_kd);
        void pid_resetCalculationValues();
        void pid_setIntegralMax(double given);

        bool pid_isComplete();
        
        double pid_getP();
        double pid_getI();
        double pid_getD();
        double pid_getError();

        double pid_calculate(double given_error);

};