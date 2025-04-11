#pragma once

#include <chrono>

#include "vex.h"

class Timer {

    private:

        std::chrono::steady_clock::time_point startTime;
        std::chrono::steady_clock::time_point endTime;
        std::chrono::steady_clock::time_point prevUpdateTime;

        double elapsedTime;
        double deltaTime;  
        double deltaRate;

    protected:

        //void updateElapsed();

    public:

        Timer();
        
        /*clock timer*/
        void timer_startCount();
        void timer_endCount();
        double timer_getElapsed();    
        
        /*delta timer --> change in time increments*/
        void timer_startDelta();
        void timer_endDelta();
        void timer_setDeltaRate(double given);
        double timer_getDelta();

};