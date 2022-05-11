#include "Controls.h"

Controls::Controls()
{
    //idk
}

double Controls::getXStrafe()
{
    double x = lJoy->GetRawAxis(InputConstants::LJOY_X);
    frc::SmartDashboard::PutNumber("X strafe", x);
    return x;
    //return lJoy->GetRawAxis(InputConstants::LJOY_X) * GeneralConstants::MAX_VOLTAGE;
}

double Controls::getYStrafe()
{
    double y = -lJoy->GetRawAxis(InputConstants::LJOY_Y);
    frc::SmartDashboard::PutNumber("Y strafe", y);
    return y;
    //return lJoy->GetRawAxis(InputConstants::LJOY_Y) * GeneralConstants::MAX_VOLTAGE;
}

double Controls::getTurn()
{
    if(climbMode_ && rJoy->GetTrigger())
    {
        return 0;
    }
    else
    {
        double turn = rJoy->GetRawAxis(InputConstants::RJOY_X);
        frc::SmartDashboard::PutNumber("Turn", turn);
        return turn;
        //return rJoy->GetRawAxis(InputConstants::RJOY_X) * (GeneralConstants::MAX_VOLTAGE * GeneralConstants::MAX_VOLTAGE / (SwerveConstants::LENGTH * (SwerveConstants::LENGTH + SwerveConstants::WIDTH * SwerveConstants::WIDTH) / 4) ); //TODO check math
    }
}

double Controls::getClimbPower() //TODO maybe change to the thumb joystick thing?
{
    if(climbMode_ && rJoy->GetTrigger())
    {
        return rJoy->GetRawAxis(InputConstants::RJOY_Y) * GeneralConstants::MAX_VOLTAGE; //TODO check if max voltage is too much
    }
    else
    {
        return 0;
    }
}

bool Controls::getClimbToggle()
{
    return lJoy->GetRawButton(InputConstants::CLIMB_TOGGLE_BUTTON);
}

bool Controls::intakePressed()
{
    return lJoy->GetTrigger();
}

bool Controls::outakePressed()
{
    return lJoy->GetRawButton(InputConstants::OUTAKE_BUTTON);
}