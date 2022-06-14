#pragma once

#include <ctre/Phoenix.h>
#include <iostream>
#include <math.h>
#include "Controls.h"
#include "Constants.h"

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

        State state_;
        bool zeroed_;
        double setPos_, prevError_, integralError_;
        double kP_ = 0.0008; //TODO tune values, also only with physics and this time it's shit
        double kI_ = 0.000;
        double kD_ = 0.0000001;
        bool atPos_;
        //0.0025, 0.0001, 0.0000001
};