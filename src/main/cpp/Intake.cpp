#include "Intake.h"

Intake::Intake() : intakeMotor_(IntakeConstants::MOTOR_ID), intakePneumatic_(frc::PneumaticsModuleType::CTREPCM, IntakeConstants::SOLENOID_ID)
{
    intakeMotor_.SetNeutralMode(NeutralMode::Coast);
    //intakePneumatic_->Set(false);

    state_ = RETRACTED_IDLE;
}

Intake::State Intake::getState(){ return state_; }

void Intake::setState(State state)
{
    /*if(state == INTAKING || state == OUTAKING)
    {
        if(state_ == RETRACTED_IDLE)
        {
            std::cout << "Deploy intake before spinning it" << std::endl;
        }
        else
        {
            state_ = state;
        }
    }
    else
    {
        state_ = state;
    }*/

    state_ = state;
}

void Intake::periodic()
{   
    switch(state_)
    {
        case RETRACTED_IDLE:
        {
            stop();
            retract();
            break;
        }
        case EXTENDED_IDLE:
        {
            stop();
            deploy();
            break;
        }
        case LOADING:
        {
            retract();
            run(true);
            break;
        }
        case INTAKING:
        {
            deploy();
            run(true);
            break;
        }
        case OUTAKING:
        {
            deploy();
            run(false);
            break;
        }
    }
}

void Intake::run(bool forward)
{
    if(forward)
    {
        intakeMotor_.SetVoltage(units::volt_t(4.68)); //TODO test value, also put in constants
    }
    else
    {
        intakeMotor_.SetVoltage(units::volt_t(-4.68));
    }
}

void Intake::stop()
{
    intakeMotor_.SetVoltage(units::volt_t(0));
}

void Intake::deploy()
{
    intakePneumatic_.Set(true); //TODO test true/false
}

void Intake::retract()
{
    intakePneumatic_.Set(false);
}