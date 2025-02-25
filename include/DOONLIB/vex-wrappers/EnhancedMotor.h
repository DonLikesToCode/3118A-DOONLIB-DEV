#pragma once

#include "vex.h"
#include "DOONLIB/motion-utils.hpp"
#include "DOONLIB/pid-util.h"

class EnhancedMotor : public vex::motor {

    private:

        double motor_cmd;
        double isForward{true};

        enum MOTOR_MODE { 
            DUTY,
            POSITION,
            VELOCITY
        };
        
        enum MOTOR_MODE controlMode{DUTY};
        
        // static auto mapToPort(int port) {
        //     switch(port) {
        //         case 1: return vex::PORT1;
        //         case 2: return vex::PORT2;
        //         case 3: return vex::PORT3;
        //         case 4: return vex::PORT4;
        //         case 5: return vex::PORT5;
        //         case 6: return vex::PORT6;
        //         case 7: return vex::PORT7;
        //         case 8: return vex::PORT8;
        //         case 9: return vex::PORT9;
        //         case 10: return vex::PORT10;
        //         case 11: return vex::PORT11;
        //         case 12: return vex::PORT12;
        //         case 13: return vex::PORT13;
        //         case 14: return vex::PORT14;
        //         case 15: return vex::PORT15;
        //         case 16: return vex::PORT16;
        //         case 17: return vex::PORT17;
        //         case 18: return vex::PORT18;
        //         case 19: return vex::PORT19;
        //         case 20: return vex::PORT20;
        //         case 21: return vex::PORT21;
        //         case 22: return vex::PORT22;
        //     default: return vex::PORT1;
        //     };
        // }

    public:

        EnhancedMotor(int port_num, vex::gearSetting gearType = vex::gearSetting::ratio18_1);
        
        void setMode(MOTOR_MODE given);
        
        void setCmd(double cmd);

        double getEncoderPosition(bool isRadians); //degrees/radians
        void setEncoderPosition(double given); //degrees
        void resetEncoderPosition(); //reset to 0

        void writeAll(void);

};