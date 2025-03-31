#pragma once

#include "vex.h"
#include "DOONLIB/utility-classes/timer.h"

class PID {

    private:

        /* Defaults */
        double controlled_cmd;
        double settleTolerance, settleTime;

        double kP, kI, kD;
        double integral, maxIntegral{0.05*kP}, derivative;

        Timer dt_pid;
        double given_dt;

        double error, preverror;

        /* Slew Rate*/
        bool isSlew;

    public:

        PID(double cmd, double tolerance, bool slew = false, double dt = 10);

        void setConstants(double sent_kp, double sent_ki, double send_kd);
        void resetCalculationValues();
        void setIntegralMax(double given);

        bool isComplete();
        
        double getP();
        double getI();
        double getD();
        double getError();

        double calculate(double given_error);

};