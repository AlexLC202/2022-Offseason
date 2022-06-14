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
        bool getPneumatic1Toggle();
        bool getPneumatic2Toggle();
        bool autoClimbPressed();

        bool intakePressed();
        bool outakePressed();

        bool shootPressed();
        double getTurretManual();

        bool increaseRange();
        bool decreaseRange();

        bool getClimbMode(){ return climbMode_; }
        void setClimbMode(bool climbMode){ climbMode_ = climbMode; }

    private:
        bool climbMode_;

        frc::Joystick lJoy_;
        frc::Joystick rJoy_;
        frc::Joystick xbox_;
};