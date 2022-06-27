#include "Climb.h"

Climb::Climb() : gearboxMaster_(ClimbConstants::MASTER_ID), gearboxSlave_(ClimbConstants::SLAVE_ID), pneumatic1_(frc::PneumaticsModuleType::CTREPCM, ClimbConstants::PNEUMATIC_1_ID), pneumatic2_(frc::PneumaticsModuleType::CTREPCM, ClimbConstants::PNEUMATIC_2_ID), brake_(frc::PneumaticsModuleType::CTREPCM, ClimbConstants::BRAKE_ID)
{
    gearboxMaster_.SetNeutralMode(NeutralMode::Brake);
    gearboxSlave_.SetNeutralMode(NeutralMode::Brake);

    gearboxSlave_.Follow(gearboxMaster_);

    state_ = IDLE;
    autoState_ = UNINITIATED;
}

Climb::State Climb::getState()
{
    return state_;
}

void Climb::setState(State state)
{
    state_ = state;
}

Climb::AutoState Climb::getAutoState()
{
    return autoState_;
}

void Climb::setAutoState(AutoState autoState)
{
    autoState_ = autoState;
}

void Climb::periodic(double pitch)
{
    pitch_ = pitch;

    switch(state_)
    {
        case IDLE:
        {
            stop();
            setBrake(true);
            break;
        }
        case DOWN:
        {
            setBrake(true);
            setPneumatics(false, false);
            stop();
            break;
        }
        case AUTO:
        {
            autoClimb();
            setBrake(false);
            break;
        }
        case MANUAL:
        {
            setBrake(false);
            break;
        }
    }
}

void Climb::setPneumatics(bool pneumatic1, bool pneumatic2)
{
    pneumatic1_.Set(!pneumatic1);
    pneumatic2_.Set(pneumatic2);
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
    frc::SmartDashboard::PutNumber("CP", gearboxMaster_.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("CC", gearboxMaster_.GetSupplyCurrent());
    gearboxMaster_.SetVoltage(units::volt_t(power)); //TODO check if slave motor spins as well
}

void Climb::stop()
{
    gearboxMaster_.SetVoltage(units::volt_t(0));
}

void Climb::autoClimb()
{
    switch(autoState_)//TODO clean up switch? Kinda redundant
    {
        case UNINITIATED:
        {
            gearboxMaster_.SetSelectedSensorPosition(0);
            setBrake(false);
            stageComplete_ = false;
            nextStage_ = false;
            autoState_ = CLIMB_LOW;
            break;
        }
        case CLIMB_LOW:
        {
            if(!stageComplete_)
            {
                if(climbBar())
                {
                    stageComplete_ = true;
                }
            }
            else if(nextStage_)
            {
                stageComplete_ = false;
                nextStage_ = false;
                autoState_ = EXTEND_TO_MID;
            }
            break;
        }
        case EXTEND_TO_MID:
        {
            if(!stageComplete_)
            {
                if(raiseToBar())
                {
                    stageComplete_ = true;
                }
            }
            else if(nextStage_)
            {
                stageComplete_ = false;
                nextStage_ = false;
                autoState_ = CLIMB_MID;
            }
            break;
        }
        case CLIMB_MID:
        {
            if(!stageComplete_)
            {
                if(climbBar())
                {
                    stageComplete_ = true;
                }
            }
            else if(nextStage_)
            {
                stageComplete_ = false;
                nextStage_ = false;
                autoState_ = EXTEND_TO_HIGH;
            }
            break;
        }
        case EXTEND_TO_HIGH:
        {
            if(!stageComplete_)
            {
                if(raiseToBar())
                {
                    stageComplete_ = true;
                }
            }
            else if(nextStage_)
            {
                stageComplete_ = false;
                nextStage_ = false;
                autoState_ = CLIMB_HIGH;
            }
            break;
        }
        case CLIMB_HIGH: //TODO, some way of figuring out when it's done other than manual cancel or all the way up
        {
            if(!stageComplete_)
            {
                if(climbBar())
                {
                    stageComplete_ = true;
                }
            }
            else
            {
                autoState_ = DONE;
            }
            break;
        }
        case DONE:
        {
            gearboxMaster_.SetVoltage(units::volt_t(0));
            setBrake(true);
            break;
        }
    }

    
}

bool Climb::stageComplete()
{
    return stageComplete_;
}

void Climb::readyNextStage()
{
    nextStage_ = true;
}

bool Climb::climbBar()
{
    if(gearboxMaster_.GetSupplyCurrent() > ClimbConstants::STALL_CURRENT)
    {
        gearboxMaster_.SetVoltage(units::volt_t(0));
        return true;
    }

    if(autoState_ == CLIMB_HIGH && gearboxMaster_.GetSelectedSensorPosition() < ClimbConstants::CLEAR_OF_BARS)
    {
        gearboxMaster_.SetVoltage(units::volt_t(0));
        return true;
    }

    gearboxMaster_.SetVoltage(units::volt_t(ClimbConstants::CLIMB_VOLTAGE)); //TODO get direction
    return false;
}

bool Climb::raiseToBar()
{
    double pos = gearboxMaster_.GetSelectedSensorPosition();
    if(pos > ClimbConstants::ABOVE_STATIC_HOOKS)
    {
        gearboxMaster_.SetVoltage(units::volt_t(ClimbConstants::SLOW_RAISE_VOLTAGE)); //TODO get direction and speed
    }
    else if(pos <= ClimbConstants::ABOVE_STATIC_HOOKS && pos > ClimbConstants::CLEAR_OF_BARS)
    {
        gearboxMaster_.SetVoltage(units::volt_t(ClimbConstants::RAISE_VOLTAGE));
    }
    else if(pos <= ClimbConstants::CLEAR_OF_BARS)
    {
        gearboxMaster_.SetVoltage(units::volt_t(-pos * kP_)); //TODO make a proper PID?
    }
    //TODO see if clamping at 6 and having two stages is better, or if quick deceleration is better

    if(autoState_ == EXTEND_TO_MID)
    {
        if(abs(pos) < ClimbConstants::EXTEND_THRESHOLD)
        {
            setPneumatics(true, true);
            return true;
        }

        if(pos < ClimbConstants::CLEAR_OF_BARS && pos > ClimbConstants::EXTEND_THRESHOLD)
        {
            setPneumatics(true, false);
        }
    }
    else if(autoState_ == EXTEND_TO_HIGH)
    {
        if(abs(pos) < ClimbConstants::EXTEND_THRESHOLD && pitch_ > ClimbConstants::PITCH_MIN && pitch_ < ClimbConstants::PITCH_MAX)
        {
            setPneumatics(true, false);
            return true;
        }
    }

    return false;
}

void Climb::setBrake(bool brake)
{
    brake_.Set(!brake);
}