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
            ZEROING, 
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
        void zero();
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
        bool zeroed_, aimed_, foundGoal_;
        double prevYaw_, yaw_, offset_, goalX_, goalY_, robotGoalAng_;

        double prevError_, integralError_;
        double tkP_ = 0.3; //TODO tune, tuned with physics and that might be bad rn
        double tkI_ = 0.0; //0.3, 0.0, 0.015 with sim
        double tkD_ = 0.015;
};