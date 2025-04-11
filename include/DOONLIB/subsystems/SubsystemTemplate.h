#pragma once

#include "vex.h"

class SubsystemTemplate {

    private:

        std::string label;

    protected:

        void SubsystemPeriodic();
        void SubsystemAutonomous();

    public:

        SubsystemTemplate(std::string SubsystemName);

        ~SubsystemTemplate();

        /*  Define any public use functions here for your subsystem 
            You should overall have functions that write to your motors
            and general logic
        */  

        /* Recommended */

        // void WriteMotors();
        // void GetSensorReading();
        

};