#include "Channel.h"

Channel::Channel()
{
    channelMotor_->SetNeutralMode(NeutralMode::Coast);
    state_ = IDLE;
}

Channel::State Channel::getState()
{
    return state_;
}
        
void Channel::setState(State state)
{
    state_ = state;
}
     
void Channel::periodic()
{
    switch(state_)
    {
        case INTAKING:
        {
            run(true);
            break;
        }
        case OUTAKING:
        {
            run(false);
            break;
        }
        case IDLE:
        {
            stop();
            break;
        }
    }
}
        
void Channel::run(bool forward)
{
    if(forward)
    {
        channelMotor_->SetVoltage(units::volt_t(4.8));
    }
    else
    {
        channelMotor_->SetVoltage(units::volt_t(-4.8));
    }
    
}
        
void Channel::stop()
{
    channelMotor_->Set(ControlMode::PercentOutput, 0);
}