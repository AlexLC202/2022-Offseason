#pragma once

#include <ctre/Phoenix.h>
#include <iostream>
#include <math.h>
#include <fstream>
#include <map>
#include "Controls.h"
#include "Constants.h"

#include "SwerveDrive.h"
#include "Limelight.h"
#include "Hood.h"
#include "Turret.h"

using namespace std;

class Shooter
{
    public:
        enum State
        {
            IDLE,
            TRACKING,
            REVING,
            UNLOADING,
            MANUAL
        };
        State getState();
        void setState(State state);

        Shooter(Limelight* limelight);
        void periodic(double yaw, SwerveDrive* swerveDrive);
        void createMap();
        void reset();

        double linVelToSensVel(double velocity);
        double calcFlyPID(double velocity);

        tuple<double, double, double> calcShootingWhileMoving(double hoodAngle, double velocity, double goalXVel, double goalYVel);
        
    private:
        Limelight* limelight_;
        WPI_TalonFX flywheelMaster_, flywheelSlave_, kickerMotor_;

        Turret turret_;
        Hood hood_;
        bool hoodZeroing_, flywheelReady_, shotReady_, hasShot_;
        bool hasMap_  = false;
        double mapPoints_ = 0;

        State state_;

        double setPos_, prevError_, integralError_;
        double fKp_ = 0.0005; //TODO tune values
        double fKi_ = 0.001; //0.0005, 0.004, 0.0
        double fKd_ = 0;

        double yaw_;

        map<double, tuple<double, double, double>> shotsMap_;
};
