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
            turretMotor_.SetNeutralMode(NeutralMode::Coast); //TODO change to brake after testing
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
        double volts = calcPID();
        frc::SmartDashboard::PutNumber("T Volts", volts);
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
            //turretMotor_.SetVoltage(volts);
        }
        
    }
    
}

double Turret::calcFeedForward()
{
    double yawVel = (yaw_ - prevYaw_) / GeneralConstants::Kdt;
    prevYaw_ = yaw_;

    double radPerSec = -(yawVel * ShooterConstants::TICKS_PER_TURRET_DEGREE * 2 * M_PI) / GeneralConstants::TICKS_PER_ROTATION;
    
    double ff = radPerSec / GeneralConstants::Kv;
    frc::SmartDashboard::PutNumber("FF", ff);
    return ff;
}

double Turret::calcError()
{
    double error, angToGoal;
    if(limelight_->hasTarget())
    {
        error = offset_ + limelight_->getXOff();
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
            //double wantedTurretAng = angToGoal + (180 - yaw_) + offset_; //TODO I think yaw offset is needed
            double wantedTurretAng = (180 - robotGoalAng_) + angToGoal + offset_;

            error =  wantedTurretAng - getAngle();
        }
        frc::SmartDashboard::PutNumber("TN/A", error);

        error = 0;
    }

    aimed_ = (abs(error) < 5); //TODO get value
    frc::SmartDashboard::PutNumber("Terror", error);
    frc::SmartDashboard::PutNumber("TPos", getAngle());
    frc::SmartDashboard::PutNumber("TTicks", turretMotor_.GetSelectedSensorPosition());

    if(abs(error + getAngle()) > 180)
    {
        double newError = (error > 0) ? error - 360 : error + 360;
        frc::SmartDashboard::PutNumber("TNError", newError);
        return newError;
    }
    else
    {
        return error;
    }
}

double Turret::calcPID()
{
    double error = calcError();
    
    double deltaError = (error - prevError_) / GeneralConstants::Kdt;
    integralError_ += error * GeneralConstants::Kdt;
    prevError_ = error;

    double power = (tkP_*error) + (tkI_*integralError_) + (tkD_*deltaError);
    //power += calcFeedForward();

    return std::clamp(power, -(double)GeneralConstants::MAX_VOLTAGE, (double)GeneralConstants::MAX_VOLTAGE); //TODO get cap value
}