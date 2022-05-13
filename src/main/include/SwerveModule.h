#pragma once

#include <ctre/Phoenix.h>
#include <iostream>
#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <string.h>
#include "Controls.h"
#include "Constants.h"

//#include <frc/MotorSafety.h>
//#include <frc/smartdashboard/SmartDashboard.h>
//#include <units/units.h>

class SwerveModule
{
    public:
        SwerveModule(int turnID, int driveID, int cancoderID, double offset);

        void periodic(double driveSpeed, double angle);
        void move(double driveSpeed, double angle);

        double calcAngPID(double setAngle);
        double calcDrivePID(double driveSpeed);
        double findError(double setAngle);

        
        double getDriveVelocity();
        double getAngle();

        void setP(double p){ akP_ = p; }
        void setD(double d){ akD_ = d; }

    private:
        WPI_TalonFX turnMotor_;
        WPI_TalonFX driveMotor_;
        WPI_CANCoder cancoder_;

        std::string id_;
        double offset_;
        int direction_ = 1;

        double aPrevError_, aIntegralError_, dPrevError_, dIntegralError_;

        double akP_ = 0.08; //TODO tune values
        double akI_ = 0.0;
        double akD_ = 0.001;

        double dkP_ = 0.0; //TODO tune values
        double dkI_ = 0.0;
        double dkD_ = 0.0;

};