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
        void getClimbModeToggle();

        bool intakePressed();
        bool outakePressed();

        bool shootPressed();

        bool getClimbMode(){ return climbMode_; }
        void setClimbMode(bool climbMode){ climbMode_ = climbMode; }

    private:
        bool climbMode_;

        frc::Joystick lJoy_;
        frc::Joystick rJoy_;
        frc::Joystick xbox_;
};