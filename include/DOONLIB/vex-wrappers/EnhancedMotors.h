#pragma once

#include <vector>

#include "vex.h"
#include "DOONLIB/motion-utils.hpp"
#include "DOONLIB/pid-util.h"

class EnhancedMotor : public vex::motor {

    private:

        double motor_cmd;
        double isReverse;
        
        enum doonlib::MOTOR_MODES controlMode{doonlib::MOTOR_MODES::DUTY};

        PID motorPID;

        double mtr_kP{1}, mtr_kI{0}, mtr_kD{0};
        
        double mtr_tolerance{1e-2 + 0.005};

    public:

        EnhancedMotor(int port_num, vex::gearSetting gearType = vex::gearSetting::ratio18_1, 
                      doonlib::MOTOR_MODES motor_mode = doonlib::MOTOR_MODES::DUTY);
        
        // virtual ~EnhancedMotor() {}

        void setMode(doonlib::MOTOR_MODES given);
        
        // void setCommand(double cmd);
        double getCommand();

        double getEncoderPosition(bool isRadians); //degrees/radians
        double getMotorVelocity();

        void setEncoderPosition(double given); //degrees
        void resetEncoderPosition(); //reset to 0

        void set(double cmd);

        // pid configs

        void setP(double kP);
        void setI(double kI);
        void setD(double kD);
    
        void setMotorPIDConstants(double kP, double kI, double kD);

        void setTolerance(double kTol);

};

class EnhancedMotorGroup {

    private:

        double motor_cmd{0};
        std::vector<EnhancedMotor*> currentMotors;

        enum doonlib::MOTOR_MODES controlMode{doonlib::MOTOR_MODES::DUTY};

        PID motorPID;

        double mtr_tolerance{1e-2 + 0.005};

    public:

        EnhancedMotorGroup(std::vector<EnhancedMotor*> motors);

        void setMode(doonlib::MOTOR_MODES given);

        double getEncoderPosition(bool isRadians);
        void setEncoderPosition(double given);
        void resetEncoderPosition();

        // void setCommand(double cmd);
        double getCommand();

        void set(double cmd);

        // pid configs

        void setP(double kP);
        void setI(double kI);
        void setD(double kD);
    
        void setGroupPIDConstants(double kP, double kI, double kD);

        void setTolerance(double kTol);

};