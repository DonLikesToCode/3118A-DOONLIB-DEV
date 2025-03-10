#pragma once

#include <vector>

#include "vex.h"
#include "DOONLIB/motion-utils.hpp"
#include "DOONLIB/pid-util.h"

class EnhancedMotor : public vex::motor {

    private:

        double motor_cmd;
        double isForward;
        
        enum doonlib::MOTOR_MODES controlMode{doonlib::MOTOR_MODES::DUTY};

        PID motorPID;

    public:

        EnhancedMotor(int port_num, vex::gearSetting gearType = vex::gearSetting::ratio18_1, 
                      doonlib::MOTOR_MODES motor_mode = doonlib::MOTOR_MODES::DUTY);
        
        // virtual ~EnhancedMotor() {}

        void setMode(doonlib::MOTOR_MODES given);
        
        // void setCommand(double cmd);
        double getCommand();

        double getEncoderPosition(bool isRadians); //degrees/radians
        void setEncoderPosition(double given); //degrees
        void resetEncoderPosition(); //reset to 0

        void writeCommand(double cmd);

};

class EnhancedMotorGroup {

    private:

        double motor_cmd{0};
        std::vector<EnhancedMotor*> currentMotors;

        enum doonlib::MOTOR_MODES controlMode{doonlib::MOTOR_MODES::DUTY};

        PID motorPID;

    public:

        EnhancedMotorGroup(std::vector<EnhancedMotor*> motors);

        void setMode(doonlib::MOTOR_MODES given);

        double getEncoderPosition(bool isRadians);
        void setEncoderPosition(double given);
        void resetEncoderPosition();

        // void setCommand(double cmd);
        double getCommand();

        void writeCommand(double cmd);

};