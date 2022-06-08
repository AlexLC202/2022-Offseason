#include "Controls.h"

Controls::Controls() : lJoy_{InputConstants::LJOY_PORT}, rJoy_{InputConstants::RJOY_PORT}, xbox_{InputConstants::XBOX_PORT}
{
    //idk
}

void Controls::periodic()
{
    if(xbox_.GetRawButtonPressed(InputConstants::CLIMB_MODE_TOGGLE_BUTTON))
    {
        climbMode_ = !climbMode_;
        getPneumatic1Toggle();
        getPneumatic2Toggle();
    }
}

double Controls::getXStrafe()
{
    double x = lJoy_.GetRawAxis(InputConstants::LJOY_X);
    if(abs(x) < 0.05)
    {
        return 0;
    }
    return (x > 0) ? (x - 0.05) / 0.95 : (x + 0.05) / 0.95;
}

double Controls::getYStrafe()
{
    double y = -lJoy_.GetRawAxis(InputConstants::LJOY_Y);
    if(abs(y) < 0.05)
    {
        return 0;
    }
    return (y > 0) ? (y - 0.05) / 0.95 : (y + 0.05) / 0.95;
}

double Controls::getTurn()
{
    /*if(climbMode_ && rJoy_.GetTrigger())
    {
        return 0;
    }
    else
    {
        double turn = rJoy_.GetRawAxis(InputConstants::RJOY_X);
        frc::SmartDashboard::PutNumber("Turn", turn);
        return turn;
        //return rJoy->GetRawAxis(InputConstants::RJOY_X) * (GeneralConstants::MAX_VOLTAGE * GeneralConstants::MAX_VOLTAGE / (SwerveConstants::LENGTH * (SwerveConstants::LENGTH + SwerveConstants::WIDTH * SwerveConstants::WIDTH) / 4) ); //TODO check math
    }*/

    double turn = rJoy_.GetRawAxis(InputConstants::RJOY_X);
    if(abs(turn) < 0.05) //TODO get value
    {
        return 0;
    }
    return (turn > 0) ? ((turn - 0.05) / 0.95) * 0.5 : ((turn + 0.05) / 0.95) * 0.5;
}

bool Controls::fieldOrient()
{
    return xbox_.GetRawButton(InputConstants::FIELD_ORIENT_BUTTON);
}

double Controls::getClimbPower() //TODO maybe change to the thumb joystick thing?
{
    /*if(climbMode_ && rJoy_.GetTrigger())
    {
        return rJoy_.GetRawAxis(InputConstants::RJOY_Y) * GeneralConstants::MAX_VOLTAGE * 0.5;
    }
    else
    {
        return 0;
    }*/

    return xbox_.GetRawAxis(InputConstants::XBOX_LJOY_Y) * 0.5 * GeneralConstants::MAX_VOLTAGE;
}

bool Controls::getPneumatic1Toggle()
{
    return xbox_.GetRawButtonPressed(InputConstants::CLIMB_PNEUMATIC1_BUTTON);
}

bool Controls::getPneumatic2Toggle()
{
    return xbox_.GetRawButtonPressed(InputConstants::CLIMB_PNEUMATIC2_BUTTON);
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