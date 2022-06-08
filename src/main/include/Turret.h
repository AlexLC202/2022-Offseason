#pragma once

#include <ctre/Phoenix.h>
#include <iostream>
#include <math.h>
#include "Controls.h"
#include "Constants.h"
#include "Limelight.h"

class Turret
{
    public:
        enum State
        {
            IDLE,
            IMMOBILE,
            TRACKING,
            MANUAL
            //FLIPPING,
        };
        State getState();
        void setState(State state);
        bool isAimed();
        double getAngle();

        Turret(Limelight* limelight);
        void periodic(double yaw, double offset, double goalX, double goalY, double robotGoalAng, bool foundGoal);
        void reset();

        void track();
        //void flip();

        double calcFeedForward();
        double calcError();
        double calcPID();
        
    private:
        WPI_TalonFX turretMotor_;
        Limelight* limelight_;

        State state_;
        bool aimed_, foundGoal_;
        double prevYaw_, yaw_, offset_, goalX_, goalY_, robotGoalAng_;

        double prevError_, integralError_;
        double tkP_ = 0.075; //TODO tune, tuned with physics only, could be more aggressive
        double tkI_ = 0.0001;  //0.06, 0.0, 0.000008
        double tkD_ = 0.000008;
};