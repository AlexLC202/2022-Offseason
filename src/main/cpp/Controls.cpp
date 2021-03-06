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
        autoClimbCancelled();
        autoClimbPressed();
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

    /*if(abs(lJoy_.GetRawAxis(InputConstants::LJOY_X)) > 0.5)
    {
        return 0.1;
    }
    else
    {
        return 0;
    }*/
}

double Controls::getYStrafe()
{
    double y = -lJoy_.GetRawAxis(InputConstants::LJOY_Y);
    if(abs(y) < 0.05)
    {
        return 0;
    }
    return (y > 0) ? (y - 0.05) / 0.95 : (y + 0.05) / 0.95;

    /*if(abs(lJoy_.GetRawAxis(InputConstants::LJOY_X)) > 0.5)
    {
        return 0;
    }
    else
    {
        return 0.1;
    }*/
}

double Controls::getTurn()
{
    //return 0;
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
    return (turn > 0) ? ((turn - 0.05) / 0.95) * 0.3 : ((turn + 0.05) / 0.95) * 0.3;
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

bool Controls::autoClimbPressed()
{
    return xbox_.GetRawButtonPressed(InputConstants::AUTO_CLIMB_BUTTON);
}

bool Controls::autoClimbCancelled()
{
    return xbox_.GetRawButtonPressed(InputConstants::AUTO_CLIMB_CANCEL);
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

bool Controls::increaseRange()
{
    return xbox_.GetRawButtonPressed(InputConstants::DISTANCE_UP_BUTTON);
}

bool Controls::decreaseRange()
{
    return xbox_.GetRawButtonPressed(InputConstants::DISTANCE_DOWN_BUTTON);
}

double Controls::getTurretManual()
{
    /*if(abs(xbox_.GetRawAxis(InputConstants::XBOX_RJOY_X)) > 0.3)
    {
        if(xbox_.GetRawAxis(InputConstants::XBOX_RJOY_X) > 0)
        {
            return 1;
        }
        else
        {
            return -1;
        }
    }*/

    return xbox_.GetRawAxis(InputConstants::XBOX_RJOY_X) * 0.3 * GeneralConstants::MAX_VOLTAGE;
}

bool Controls::resetUnload()
{
    return xbox_.GetTrigger();
    cout << "resetting unload" << endl;
}

bool Controls::manuallyOverrideTurret()
{
    return xbox_.GetTrigger();
}
