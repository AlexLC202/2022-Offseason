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
        enum pneumaticState
        {
            DOWN,
            HALF_UP,
            UP
        };
        enum extendingState
        {
            BRAKED,
            UNBRAKED,
            MOVING
        };

    pneumaticState getPneumaticState();
    extendingState getExtendingState();

    void setPneumaticState(pneumaticState pneumaticState);
    void setExtendingState(extendingState extendingState);

    Climb();
    void periodic(Controls* controls);

    void setPneumatics(bool pneumatic1, bool pneumatic2);
    void extendArms(double power);
    void stop();

    private:

        WPI_TalonFX* gearboxMaster_ = new WPI_TalonFX(ClimbConstants::MASTER_ID);
        WPI_TalonFX* gearboxSlave_ = new WPI_TalonFX(ClimbConstants::SLAVE_ID);

        frc::Solenoid* pneumatic1_ = new frc::Solenoid(frc::PneumaticsModuleType::CTREPCM, ClimbConstants::PNEUMATIC_1_ID);
        frc::Solenoid* pneumatic2_ = new frc::Solenoid(frc::PneumaticsModuleType::CTREPCM, ClimbConstants::PNEUMATIC_2_ID);

        frc::Solenoid* brake_ = new frc::Solenoid(frc::PneumaticsModuleType::CTREPCM, ClimbConstants::BRAKE_ID);

        pneumaticState pneumaticState_;
        extendingState extendingState_;
};