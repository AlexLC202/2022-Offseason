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
            MOVING,
            ZEROING,
            AIMED
        };
        State getState();
        void setState(State state);

        Hood();
        void periodic();

        void zero();
        void setWantedPos(double setPos);
        void moveToPos();
        
        double calcPID();
        void resetPID();

    private:
        WPI_TalonFX* hood_ = new WPI_TalonFX(ShooterConstants::HOOD_ID);
        double pos_, setPos_;
        double kP_ = 0.0;
        double kI_ = 0.0;
        double kD_ = 0.0;
        
};