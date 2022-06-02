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
        double kP_ = 0.00005; //TODO tune values
        double kI_ = 0.0;
        double kD_ = 0.0;

        bool atPos_;
        
};