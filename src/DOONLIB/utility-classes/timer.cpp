#include "DOONLIB/utility-classes/timer.h"

Timer::Timer()
    : startTime(std::chrono::steady_clock::now())
    , prevUpdateTime(startTime)
    , elapsedTime(0)
    , deltaTime(0)
    , deltaRate(1)  
{}

/* clock based */

void Timer::timer_startCount() { startTime = std::chrono::steady_clock::now(); }

void Timer::timer_endCount() { endTime = std::chrono::steady_clock::now(); }

double Timer::timer_getElapsed() { 
    std::chrono::duration<double> elapsed = endTime - startTime; 
    return elapsed.count();
}

/* delta based */

void Timer::timer_startDelta() { prevUpdateTime = std::chrono::steady_clock::now(); }

void Timer::timer_endDelta() {
    auto curTime = std::chrono::steady_clock::now();
    deltaTime = std::chrono::duration<double>(curTime - prevUpdateTime).count() * deltaRate;
    prevUpdateTime = curTime;
}

void Timer::timer_setDeltaRate(double given) { deltaRate = given; }

double Timer::timer_getDelta() {
    // auto now = std::chrono::steady_clock::now();
    // deltaTime = std::chrono::duration<double>(now - startTime).count() * deltaRate;
    return deltaTime;
}