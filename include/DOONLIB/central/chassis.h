#pragma once

#include "vex.h"

//#include "DOONLIB/central/doon_api.h"
#include "DOONLIB/vex-wrappers/EnhancedMotors.h"
#include "DOONLIB/utility-classes/pose.h"
#include "DOONLIB/motion-utils.hpp"


class DriveChassis {

    private:

        doonlib::DRIVE_MODES driveMode;

        // vex::controller& Controller;

        EnhancedMotorGroup* leftDT;
        EnhancedMotorGroup* rightDT;

        // double LeftAxisInput;
        // double RightAxisInput;

        enum ODOM_DRIVE_MODES {
            RAMSETE,
            BOOMERANG,
            RAM_PID,
            PID
        };

        // double horizontalTrackingDiameter;
        // double verticalTrackingDiameter;

        vex::rotation* horzTracker;
        vex::rotation* vertTracker; 

        vex::inertial* inertial;

        static Pose robotPose;

        /* Chassis Configs */
        std::vector<double> chassis_configs = {
            1.0, // Gear Ratio [0]
            200, // Drive RPM [1]
            12.0, // Track Width [2]
            4.0, // Drive Diameter [3]  
            2.0, // Vertical Tracking Diameter [4]
            2.0, // Horiztonal Tracking Diameter [5]
        };

        // std::vector<double> ramsete_configs = {
        //     1.0, // b term [0] --> proportional term
        //     0.0, // zeta term (ζ) [1] --> derivative term
        //     1.0, // v_d [2] --> can be tuning or computed path constant
        //     1.0, // w_d [3] --> can be tuning or computed path constant
        // };

    protected:
        
        double localX{0};
        double localY{0};
        double localTheta{0};

        double leftOffset{0};
        double rightOffset{0};
        double strafeOffset{0};

        double leftDist{0};
        double rightDist{0};
        double strafeDist{0};

        double prevLeftDist{0};
        double prevRightDist{0};
        double prevStrafeDist{0};

        double rotation{0};

        double RamseteControl(double x, double y, double heading);

    public:

        DriveChassis();

        DriveChassis(
                     EnhancedMotorGroup* LeftMotors, 
                     EnhancedMotorGroup* RightMotors,
                     vex::rotation* odomPod_vert1, 
                     vex::rotation* odomPod_horz,
                     vex::inertial* imu
        );

        /* Command Drive */

        void ArcadeDrive(double leftAxis, double rightAxis);

        void TankDrive(double leftAxis, double rightAxis);

        void CurvatureDrive();

        /* Odometry */

        void initOdometry();
        void setPose(double x, double y, double heading);

        void setPodOffsets(double leftPodOffset, double rightPodOffset, double strafePodOffset);
        void configureChassisConstants(
             double gearRatio,
             double rpm,
             double trackWidth,
             double wheelDiameter,
             double verticalPodDiameter, 
             double strafePodDiameters
        );

        // void configureRamseteConstants(
        //      double kB,
        //      double kZ, 
        //      double v_d,
        //      double w_d
        // );
        //void setRamsetePathVelocity(double v_d, double w_d);

        //float, double --> 0.0, float 

        double getPose_x();
        double getPose_y();
        double getPose_theta(bool isRadians);

        double updateOdometry();

        std::vector<double> getPose();

        /* Auton Functions */

        void driveToPose(double x, double y, double heading);

};