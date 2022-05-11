#pragma once

#include <ctre/Phoenix.h>
#include <iostream>
#include <math.h>
#include "Controls.h"
#include "Constants.h"

#include "Limelight.h"
#include "Hood.h"
#include "Turret.h"

class Shooter
{
    public:
        enum State
        {
            IDLE,
            TRACKING,
            AIMING,
            SHOOTING,
            UNLOADING,
            MANUAL
        };
        State getState();
        void setState(State state);

        Shooter();
        void periodic();

        
    private:
        Limelight limelight;
        Turret turret;
        Hood hood;
        bool hoodZeroing;
};

