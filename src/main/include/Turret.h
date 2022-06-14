#pragma once

#include <ctre/Phoenix.h>
#include <iostream>
#include <math.h>
#include "Controls.h"
#include "Constants.h"
#include "Limelight.h"
#include "Helpers.h"

class Turret
{
    public:
        enum State
        {
            IDLE,
            IMMOBILE,
            TRACKING,
            UNLOADING,
            MANUAL
            //FLIPPING,
        };
        State getState();
        void setState(State state);
        void setManualVolts(double manualVolts);
        bool isAimed();
        bool unloadReady();
        double getAngle();

        Turret(Limelight* limelight);
        void periodic(double yaw, double offset, double robotGoalAng, bool foundGoal, double x, double y);
        void reset();

        void track();
        void calcUnloadAng();
        //void flip();

        double calcFeedForward();
        double calcError();
        double calcPID();
        
    private:
        WPI_TalonFX turretMotor_;
        Limelight* limelight_;

        State state_;
        double manualVolts_;
        bool aimed_, unloadReady_, foundGoal_;
        double prevYaw_, yaw_, offset_, robotGoalAng_, x_, y_, unloadAngle_;

        double prevError_, integralError_;
        double tkP_ = 0.2; //TODO tune, tuned with physics only, could be more aggressive
        double tkI_ = 0.0001;  //0.075, 0.0001, 0.000008
        double tkD_ = 0.000008;  //0.3, 0.0001, 0.00005
};