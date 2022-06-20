#pragma once

#include <ctre/Phoenix.h>
#include <iostream>
#include <math.h>
#include "Controls.h"
#include "Constants.h"

#include <frc/Solenoid.h>

class Climb
{
    public:
        enum State
        {
            IDLE,
            DOWN,
            AUTO,
            MANUAL
        };

        enum AutoState
        {
            UNINITIATED,
            CLIMB_LOW,
            EXTEND_TO_MID,
            CLIMB_MID, 
            EXTEND_TO_HIGH,
            CLIMB_HIGH,
            DONE
        };

    State getState();
    void setState(State state);

    AutoState getAutoState();
    void setAutoState(AutoState autoState);

    Climb();
    void periodic(double pitch);

    void setPneumatics(bool pneumatic1, bool pneumatic2);
    void togglePneumatic1();
    void togglePneumatic2();
    void extendArms(double power);
    void stop();
    void autoClimb();
    bool stageComplete();
    void readyNextStage();

    bool climbBar();
    bool raiseToBar();

    void setBrake(bool brake);

    private:

        WPI_TalonFX gearboxMaster_;
        WPI_TalonFX gearboxSlave_;

        frc::Solenoid pneumatic1_;
        frc::Solenoid pneumatic2_;

        frc::Solenoid brake_;

        State state_;
        AutoState autoState_;

        double pitch_;
        bool nextStage_, stageComplete_;

        double kP_ = 0.0001;
};