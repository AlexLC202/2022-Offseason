#include "Climb.h"

Climb::Climb() : gearboxMaster_(ClimbConstants::MASTER_ID), gearboxSlave_(ClimbConstants::SLAVE_ID), pneumatic1_(frc::PneumaticsModuleType::CTREPCM, ClimbConstants::PNEUMATIC_1_ID), pneumatic2_(frc::PneumaticsModuleType::CTREPCM, ClimbConstants::PNEUMATIC_2_ID), brake_(frc::PneumaticsModuleType::CTREPCM, ClimbConstants::BRAKE_ID)
{
    gearboxMaster_.SetNeutralMode(NeutralMode::Brake);
    gearboxSlave_.SetNeutralMode(NeutralMode::Brake);

    gearboxSlave_.Follow(gearboxMaster_);

}

Climb::pneumaticState Climb::getPneumaticState()
{
    return pneumaticState_;
}

void Climb::setPneumaticState(pneumaticState pneumaticState)
{
    pneumaticState_ = pneumaticState;
}

void Climb::periodic()
{
    /*switch(pneumaticState_) //TODO AUTOCLIMB WOOOOOOO
    {
        case DOWN:
        {
            setPneumatics(false, false);
            break;
        }
        case HALF_UP:
        {
            setPneumatics(true, false);
            break;
        }
        case UP:
        {
            setPneumatics(true, true);
            break;
        }
    }*/
}

void Climb::setPneumatics(bool pneumatic1, bool pneumatic2)
{
    pneumatic1_.Set(pneumatic1);
    pneumatic2_.Set(!pneumatic2);
}

void Climb::togglePneumatic1()
{
    pneumatic1_.Toggle();
}

void Climb::togglePneumatic2()
{
    pneumatic2_.Toggle();
}

void Climb::extendArms(double power)
{
    //std::cout << power << std::endl;
    std::cout << gearboxMaster_.GetSelectedSensorPosition() << std::endl;
    gearboxMaster_.SetVoltage(units::volt_t(power)); //TODO check if slave motor spins as well
}

void Climb::stop()
{
    gearboxMaster_.SetVoltage(units::volt_t(0));
}

void Climb::setBrake(bool brake)
{
    brake_.Set(!brake);
}