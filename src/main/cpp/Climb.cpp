#include "Climb.h"

Climb::Climb()
{
    gearboxMaster_->SetNeutralMode(NeutralMode::Brake);
    gearboxSlave_->SetNeutralMode(NeutralMode::Brake);

    gearboxSlave_->Follow(*gearboxMaster_);

    /*pneumatic1_->Set(true);
    pneumatic2_->Set(false);
    brake_->Set(false);*/

    pneumaticState_ = HALF_UP;
    extendingState_ = BRAKED;

}

Climb::pneumaticState Climb::getPneumaticState()
{
    return pneumaticState_;
}
    
Climb::extendingState Climb::getExtendingState()
{
    return extendingState_;
}

void Climb::setPneumaticState(pneumaticState pneumaticState)
{
    pneumaticState_ = pneumaticState;
}

void Climb::setExtendingState(extendingState extendingState)
{
    if(extendingState == MOVING)
    {
        if(extendingState_ == BRAKED)
        {
            std::cout << "Brake was on, turning that off first" << std::endl;
            extendingState_ = UNBRAKED;
        }
    }
    else
    {
        extendingState_ = extendingState;
    }
    
}

void Climb::periodic(Controls* controls)
{
    switch(pneumaticState_)
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
    }

    switch(extendingState_)
    {
        case BRAKED:
        {
            stop();
            brake_->Set(false);
            break;
        }
        case UNBRAKED:
        {
            stop();
            brake_->Set(true);
            break;
        }
        case MOVING:
        {
            extendArms(controls->getClimbPower());
            break;
        }
    }
}

void Climb::setPneumatics(bool pneumatic1, bool pneumatic2)
{
    pneumatic1_->Set(pneumatic1);
    pneumatic2_->Set(pneumatic2);
}

void Climb::extendArms(double power)
{
    gearboxMaster_->SetVoltage(units::volt_t(power)); //TODO check if slave motor spins as well
}

void Climb::stop()
{
    gearboxMaster_->Set(ControlMode::PercentOutput, 0);
}