#pragma once

#include <frc/Joystick.h>
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>

class Controls
{
    public:
        Controls();
        void periodic();

        double getXStrafe();
        double getYStrafe();
        double getTurn();

        bool fieldOrient();

        double getClimbPower();
        bool getClimbToggle();

        bool intakePressed();
        bool outakePressed();

        bool getClimbMode(){ return climbMode_; }
        void setClimbMode(bool climbMode){ climbMode_ = climbMode; }

    private:
        bool climbMode_;

        frc::Joystick lJoy;
        frc::Joystick rJoy;
        frc::Joystick xbox;
};