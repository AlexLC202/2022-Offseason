#pragma once

#include <frc/Joystick.h>
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>

class Controls
{
    public:
        Controls();

        double getXStrafe();
        double getYStrafe();
        double getTurn();

        double getClimbPower();
        bool getClimbToggle();

        bool intakePressed();
        bool outakePressed();

        bool getClimbMode(){ return climbMode_; }
        void setClimbMode(bool climbMode){ climbMode_ = climbMode; }

    private:
        bool climbMode_;

        frc::Joystick* lJoy = new frc::Joystick(InputConstants::LJOY_PORT);
        frc::Joystick* rJoy = new frc::Joystick(InputConstants::RJOY_PORT);
};