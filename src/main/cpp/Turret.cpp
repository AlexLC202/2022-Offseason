#include "Turret.h"

Turret::Turret(Limelight* limelight) : turretMotor_(ShooterConstants::TURRET_ID)
{
    turretMotor_.SetNeutralMode(NeutralMode::Coast);
    reset();
    limelight_ = limelight;

    state_ = IDLE;
}

void Turret::periodic(double yaw, double offset, double goalX, double goalY, bool foundGoal)
{
    yaw_ = yaw;
    offset_ = offset;
    goalX_ = goalX;
    goalY_ = goalY;
    foundGoal_ = foundGoal;

    if(!zeroed_)
    {
        state_ = ZEROING;
    }

    switch(state_)
    {
        case IDLE:
        {
            turretMotor_.SetVoltage(units::volt_t(0));
            turretMotor_.SetNeutralMode(NeutralMode::Coast);
            break;
        }
        case IMMOBILE:
        {
            turretMotor_.SetVoltage(units::volt_t(0));
            turretMotor_.SetNeutralMode(NeutralMode::Brake);
            break;
        }
        case ZEROING:
        {
            turretMotor_.SetNeutralMode(NeutralMode::Brake);
            zero();
            break;
        }
        case TRACKING:
        {
            turretMotor_.SetNeutralMode(NeutralMode::Brake);
            track();
            break;
        }
        case MANUAL:
        {

            break;
        }
        /*case FLIPPING: //TODO test if even needed
        {
            turretMotor_.SetNeutralMode(NeutralMode::Brake);
            flip();
            break;
        }*/
    }
}

Turret::State Turret::getState()
{
    return state_;
}

void Turret::setState(State state)
{
    if(state == ZEROING)
    {
        zeroed_ = false;
    }
    state_ = state;
}

bool Turret::isAimed()
{
    return aimed_;
}

double Turret::getAngle()
{
    return turretMotor_.GetSelectedSensorPosition() / ShooterConstants::TICKS_PER_TURRET_DEGREE;
}

void Turret::zero()
{
    turretMotor_.SetVoltage(units::volt_t(-1)); //TODO test direction and speed

    if(turretMotor_.GetSupplyCurrent() > ShooterConstants::TURRET_ZERO_CURRENT)
    {
        turretMotor_.SetVoltage(units::volt_t(0));
        reset();
        state_ = IMMOBILE;
    }
}

void Turret::reset()
{
    turretMotor_.SetSelectedSensorPosition(0);
    zeroed_ = true;
}

void Turret::track()
{
    if(!limelight_->hasTarget() && !foundGoal_)
    {
        turretMotor_.SetVoltage(units::volt_t(0));
    }
    else
    {
        units::volt_t volts(calcPID());
        turretMotor_.SetVoltage(volts);
    }
    
}

double Turret::calcFeedForward()
{
    double yawVel = (yaw_ - prevYaw_) / GeneralConstants::Kdt;

    double radPerSec = -(yawVel * ShooterConstants::TICKS_PER_TURRET_DEGREE * 2 * M_PI) / GeneralConstants::TICKS_PER_ROTATION;
    
    return radPerSec / GeneralConstants::Kv;
}

double Turret::calcError()
{
    double error;
    if(limelight_->hasTarget())
    {
        error = offset_ + limelight_->getXOff();
    }
    else
    {
        double angToGoal = -(atan2(-goalY_, -goalX_) * 180 / M_PI) + 90;
        double wantedTurretAng = angToGoal + (180 - yaw_) + offset_;

        error =  wantedTurretAng - getAngle();
    }

    aimed_ = (abs(error) < 5); //TODO get value

    if(abs(error + getAngle()) > 180)
    {
        return (error > 0) ? error - 360 : error + 360;
    }
    else
    {
        return error;
    }
}

double Turret::calcPID()
{
    double error = -calcError();
    
    double deltaError = (error - prevError_) / GeneralConstants::Kdt;
    integralError_ += error * GeneralConstants::Kdt;
    prevError_ = error;

    double power = (tkP_*error) + (tkI_*integralError_) + (tkD_*deltaError) + calcFeedForward();

    return std::clamp(power, -(double)GeneralConstants::MAX_VOLTAGE * 0.5, (double)GeneralConstants::MAX_VOLTAGE * 0.5); //TODO get cap value
}