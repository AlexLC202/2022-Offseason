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
        /*enum pneumaticState
        {
            DOWN,
            HALF_UP,
            UP
        };

    pneumaticState getPneumaticState();
    void setPneumaticState(pneumaticState pneumaticState);*/

    Climb();
    void periodic();

    void setPneumatics(bool pneumatic1, bool pneumatic2);
    void togglePneumatic1();
    void togglePneumatic2();
    void extendArms(double power);
    void stop();

    void setBrake(bool brake);

    private:

        WPI_TalonFX gearboxMaster_;
        WPI_TalonFX gearboxSlave_;

        frc::Solenoid pneumatic1_;
        frc::Solenoid pneumatic2_;

        frc::Solenoid brake_;

        //pneumaticState pneumaticState_;
};