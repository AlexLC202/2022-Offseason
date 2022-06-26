#include "AutoPaths.h"

void AutoPaths::setPath(Path path)
{
    path_ = path;
}


AutoPaths::Path AutoPaths::getPath()
{
    return path_;
}

Shooter::State AutoPaths::getShooterState()
{
    return shooterState_;
}

Intake::State AutoPaths::getIntakeState()
{
    return intakeState_;
}

void AutoPaths::startTimer()
{
    timer_.Start();
}

void AutoPaths::stopTimer()
{
    timer_.Stop();
}

void AutoPaths::periodic(SwerveDrive* swerveDrive)
{
    switch(path_)
    {
        case TAXI_DUMB:
        {
            intakeState_ = Intake::RETRACTED_IDLE;
            shooterState_ = Shooter::IDLE;
            if(timer_.Get().value() < 2.0) //TODO get value
            {
                swerveDrive->drive(0, -0.2, 0);
            }
            else
            {
                swerveDrive->drive(0, 0, 0);
            }
            break;
        }
        case TWO_DUMB:
        {
            if(timer_.Get().value() < 2.0) //TODO get value
            {
                intakeState_ = Intake::INTAKING;
                shooterState_ = Shooter::TRACKING;
                swerveDrive->drive(0, -0.2, 0);
            }
            else
            {
                swerveDrive->drive(0, 0, 0);
                shooterState_ = Shooter::REVING;
            }
            break;
        }
        case TWO_RIGHT:
        {
            break;
        }
        case TWO_MIDDLE:
        {
            break;
        }
        case TWO_LEFT:
        {
            break;
        }
        case THREE:
        {
            break;
        }
        case FIVE:
        {
            break;
        }
    }
}

double AutoPaths::initYaw()
{
    switch(path_)
    {
        case TAXI_DUMB:
        {
            return 0;
            break;
        }
        case TWO_DUMB:
        {
            return 0;
            break;
        }
        case TWO_RIGHT:
        {
            return 90;
            break;
        }
        case TWO_MIDDLE:
        {
            return 135;
            break;
        }
        case TWO_LEFT:
        {
            return -135;
            break;
        }
        case THREE:
        {
            return  90;
            break;
        }
        case FIVE:
        {
            return 90;
            break;
        }
    }

    return 0;
}