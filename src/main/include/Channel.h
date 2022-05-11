#pragma once

#include <ctre/Phoenix.h>
#include <iostream>
#include <math.h>
#include "Controls.h"
#include "Constants.h"

class Channel
{
    public:
        enum State
        {
            INTAKING, 
            OUTAKING,
            IDLE
        };

        State getState();
        void setState(State state);

        Channel();
        void periodic();
        void run(bool forward);
        void stop();

    private:
        WPI_TalonFX* channelMotor_ = new WPI_TalonFX(ChannelConstants::MOTOR_ID);
        State state_;
};