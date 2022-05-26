#include "Controls.h"

Controls::Controls() : lJoy_{InputConstants::LJOY_PORT}, rJoy_{InputConstants::RJOY_PORT}, xbox_{InputConstants::XBOX_PORT}
{
    //idk
}

void Controls::periodic()
{
    getClimbModeToggle();
}

double Controls::getXStrafe()
{
    double x = lJoy_.GetRawAxis(InputConstants::LJOY_X);
    frc::SmartDashboard::PutNumber("X strafe", x);
    return x;
    //return lJoy_->GetRawAxis(InputConstants::LJOY_X) * GeneralConstants::MAX_VOLTAGE;
}

double Controls::getYStrafe()
{
    double y = -lJoy_.GetRawAxis(InputConstants::LJOY_Y);
    frc::SmartDashboard::PutNumber("Y strafe", y);
    return y;
    //return lJoy_->GetRawAxis(InputConstants::LJOY_Y) * GeneralConstants::MAX_VOLTAGE;
}

double Controls::getTurn()
{
    if(climbMode_ && rJoy_.GetTrigger())
    {
        return 0;
    }
    else
    {
        double turn = rJoy_.GetRawAxis(InputConstants::RJOY_X);
        frc::SmartDashboard::PutNumber("Turn", turn);
        return turn;
        //return rJoy->GetRawAxis(InputConstants::RJOY_X) * (GeneralConstants::MAX_VOLTAGE * GeneralConstants::MAX_VOLTAGE / (SwerveConstants::LENGTH * (SwerveConstants::LENGTH + SwerveConstants::WIDTH * SwerveConstants::WIDTH) / 4) ); //TODO check math
    }
}

bool Controls::fieldOrient()
{
    return lJoy_.GetRawButton(InputConstants::FIELD_ORIENT_ID);
}

double Controls::getClimbPower() //TODO maybe change to the thumb joystick thing?
{
    if(climbMode_ && rJoy_.GetTrigger())
    {
        return rJoy_.GetRawAxis(InputConstants::RJOY_Y) * GeneralConstants::MAX_VOLTAGE * 0.5; //TODO check if max voltage is too much
    }
    else
    {
        return 0;
    }
}

void Controls::getClimbModeToggle()
{
    if(xbox_.GetRawButtonPressed(InputConstants::CLIMB_TOGGLE_BUTTON))
    {
        climbMode_ = !climbMode_;
    }
}

bool Controls::intakePressed()
{
    return rJoy_.GetTrigger();
}

bool Controls::outakePressed()
{
    return rJoy_.GetRawButton(InputConstants::OUTAKE_BUTTON);
}

bool Controls::shootPressed()
{
    return lJoy_.GetTrigger();
}