#pragma once

#include <ctre/Phoenix.h>
#include <iostream>
#include <math.h>
#include "Controls.h"
#include "Constants.h"
#include "TrajectoryCalc.h"

class Hood
{
    public:
        enum State
        {
            IDLE, 
            IMMOBILE,
            AIMING,
            ZEROING,
        };
        State getState();
        void setState(State state);
        void setPID(double p, double i, double d);
        double getHoodTicks();

        bool isReady();

        Hood();
        void periodic();

        void reset();
        void zero();
        void setWantedPos(double setPos);
        void move();
        
        double calcPID();
        double angleToTicks(double angle);
        //void resetPID();

    private:
        WPI_TalonFX hoodMotor_;

        double maxV = 0;
        double maxA = 0;
        double kP = 0;
        double kD = 0;
        double kV = 0;
        double kA = 0;
        TrajectoryCalc trajectoryCalc_;
        bool initTrajectory_;
        double setTrajectoryPos_;

        State state_;
        bool zeroed_;
        double setPos_, prevError_, integralError_;
        double kP_ = 0.0008; //TODO tune values, also only with physics and this time it's shit
        double kI_ = 0.00;
        double kD_ = 0.0;
        bool atPos_;
        //0.0008, 0.000, 0.0000001
        //0.0025, 0.0001, 0.0000001
};