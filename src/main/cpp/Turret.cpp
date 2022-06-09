#include "Turret.h"

Turret::Turret(Limelight* limelight) : turretMotor_(ShooterConstants::TURRET_ID)
{
    turretMotor_.SetNeutralMode(NeutralMode::Coast);
    reset();
    limelight_ = limelight;

    state_ = IDLE;
}

void Turret::periodic(double yaw, double offset, double goalX, double goalY, double robotGoalAng, bool foundGoal)
{
    yaw_ = yaw;
    offset_ = offset;
    goalX_ = goalX;
    goalY_ = goalY;
    robotGoalAng_ = robotGoalAng;
    foundGoal_ = foundGoal;

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
    }
}

Turret::State Turret::getState()
{
    return state_;
}

void Turret::setState(State state)
{
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

void Turret::reset()
{
    turretMotor_.SetSelectedSensorPosition(0);
}

void Turret::track()
{
    if(!limelight_->hasTarget() && !foundGoal_)
    {
        turretMotor_.SetVoltage(units::volt_t(0));
    }
    else
    {
        double volts = calcPID();
        //frc::SmartDashboard::PutNumber("T Volts", volts);
        if(volts > 0 && (turretMotor_.GetSelectedSensorPosition() > (180 * ShooterConstants::TICKS_PER_TURRET_DEGREE)))
        {
            std::cout << "trying to decapitate itself" << std::endl;
            turretMotor_.SetVoltage(units::volt_t(0));
        }
        else if (volts < 0 && turretMotor_.GetSelectedSensorPosition() < (-180 * ShooterConstants::TICKS_PER_TURRET_DEGREE))
        {
            std::cout << "trying to decapitate itself" << std::endl;
            turretMotor_.SetVoltage(units::volt_t(0));
        }
        else
        {
            //turretMotor_.SetVoltage(units::volt_t(volts));
        }
        
    }
    
}

double Turret::calcFeedForward()
{
    double deltaYaw = yaw_ - prevYaw_;
    if(abs(deltaYaw) > 300)
    {
        deltaYaw = (deltaYaw > 0) ? deltaYaw - 360 : deltaYaw + 360;
    }

    double yawVel = deltaYaw / GeneralConstants::Kdt;
    prevYaw_ = yaw_;

    double radPerSec = -(yawVel * ShooterConstants::TICKS_PER_TURRET_DEGREE * 2 * M_PI) / GeneralConstants::TICKS_PER_ROTATION;
    
    double ff = radPerSec / GeneralConstants::Kv;
    return ff;
}

double Turret::calcError()
{
    double error, angToGoal;
    if(limelight_->hasTarget())
    {
        error = offset_ + limelight_->getAdjustedX();
        frc::SmartDashboard::PutNumber("TA", getAngle());
    }
    else
    {
        if(goalX_ == 0 && goalY_ == 0)
        {
            error = 0;
        }
        else
        {
            angToGoal = -(atan2(-goalY_, -goalX_) * 180 / M_PI) + 90;
            double wantedTurretAng = (180 - robotGoalAng_) + angToGoal + offset_;
            
            wantedTurretAng += 360 * 10;
            wantedTurretAng = ((int)floor(wantedTurretAng) % 360) + (wantedTurretAng - floor(wantedTurretAng));
            wantedTurretAng -= 360 * floor(wantedTurretAng / 360 + 0.5);

            error =  wantedTurretAng - getAngle();
        }
    }

    frc::SmartDashboard::PutNumber("Terror", error);

    if(abs(error + getAngle()) > 180)
    {
        error = (error > 0) ? error - 360 : error + 360;
    }

    aimed_ = (abs(error) < 3); //TODO get value
    return error;
}

double Turret::calcPID()
{
    double error = calcError();
    
    double deltaError = (error - prevError_) / GeneralConstants::Kdt;
    integralError_ += error * GeneralConstants::Kdt;

    if(abs(error) < 1)
    {
        integralError_ = 0;
    }

    if(abs(prevError_) < 3) //TODO get value, probably same as above
    {
        deltaError = 0;
    } 
    prevError_ = error;

    double power = (tkP_*error) + (tkI_*integralError_) + (tkD_*deltaError);
    //calcFeedForward();
    power += calcFeedForward();

    return std::clamp(power, -(double)GeneralConstants::MAX_VOLTAGE * 0.3, (double)GeneralConstants::MAX_VOLTAGE * 0.3); //TODO get cap value
}