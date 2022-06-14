#include "Turret.h"

Turret::Turret(Limelight* limelight) : turretMotor_(ShooterConstants::TURRET_ID)
{
    turretMotor_.SetNeutralMode(NeutralMode::Coast);
    reset();
    limelight_ = limelight;

    state_ = IDLE;
}

void Turret::periodic(double yaw, double offset, double robotGoalAng, bool foundGoal, double x, double y)
{
    yaw_ = yaw;
    offset_ = offset;
    robotGoalAng_ = robotGoalAng;
    foundGoal_ = foundGoal;
    x_ = x;
    y_ = y;

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
            turretMotor_.SetNeutralMode(NeutralMode::Coast); //TODO change
            track();
            break;
        }
        case UNLOADING:
        {
            turretMotor_.SetNeutralMode(NeutralMode::Coast); //TODO change
            calcUnloadAng();
            track();
            break;
        }
        case MANUAL:
        {
            turretMotor_.SetVoltage(units::volt_t(manualVolts_));
            calcError(); //TODO remove, just for printing values
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

void Turret::setManualVolts(double manualVolts)
{
    manualVolts_ = manualVolts;
}

bool Turret::isAimed()
{
    return aimed_;
}

bool Turret::unloadReady()
{
    return unloadReady_;
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

void Turret::calcUnloadAng()
{
    double angToHangar = 0;
    double xDist = x_ - GeneralConstants::HANGAR_X;
    double yDist = y_ - GeneralConstants::HANGAR_Y;
    if(xDist != 0 || yDist != 0)
    {
        angToHangar = -(atan2(yDist, xDist) * 180 / M_PI) - 90;
    }

    double angToGoal = 0;
    if(x_ != 0 || y_ != 0)
    {
        angToGoal = -(atan2(y_, x_) * 180 / M_PI) - 90;
    }

    angToHangar += 360 * 10; //TODO make a global function to normalize angles?
    angToHangar = ((int)floor(angToHangar) % 360) + (angToHangar - floor(angToHangar));
    //angToHangar -= 360 * floor(angToHangar / 360 + 0.5);

    angToGoal += 360 * 10; //TODO make a global function to normalize angles?
    angToGoal = ((int)floor(angToGoal) % 360) + (angToGoal - floor(angToGoal));
    //angToGoal -= 360 * floor(angToGoal / 360 + 0.5);

    if(abs(angToHangar - angToGoal) < 10) //TODO get value
    {
        angToHangar += (angToHangar > angToGoal) ? 10 : -10; //TODO I have brain damage, come up with a solution that actually works for all situations later
    }
    //Helpers::normalizeAngle(angToHangar);

    unloadAngle_ = angToHangar - yaw_;
    Helpers::normalizeAngle(unloadAngle_);
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
    double error;
    if(state_ == UNLOADING)
    {
        error = unloadAngle_ - getAngle();
    }
    else if(limelight_->hasTarget())
    {
        error = offset_ + limelight_->getAdjustedX() + LimelightConstants::TURRET_ANGLE_OFFSET;
    }
    else
    {
        double wantedTurretAng = (180 - robotGoalAng_) + offset_;
        Helpers::normalizeAngle(wantedTurretAng);

        error =  wantedTurretAng - getAngle();
    }

    frc::SmartDashboard::PutNumber("TA", getAngle());
    frc::SmartDashboard::PutNumber("Terror", error);

    if(abs(error + getAngle()) > 180)
    {
        error = (error > 0) ? error - 360 : error + 360;
    }

    return error;
}

double Turret::calcPID()
{
    double error = calcError();
    
    double deltaError = (error - prevError_) / GeneralConstants::Kdt;
    integralError_ += error * GeneralConstants::Kdt;

    if(abs(prevError_) < 2.5 && abs(error > 5)) //TODO get value, probably same as above
    {
        deltaError = 0;
        integralError_ = 0;
    } 
    prevError_ = error;

    aimed_ = (abs(error) < 3 && abs(deltaError) < 1); //TODO get value, change back to 2.5
    unloadReady_ = (abs(error) < 10); //TODO get value

    double power = (tkP_*error) + (tkI_*integralError_) + (tkD_*deltaError);
    //calcFeedForward();
    power += calcFeedForward();

    return std::clamp(power, -(double)GeneralConstants::MAX_VOLTAGE * 0.3, (double)GeneralConstants::MAX_VOLTAGE * 0.3); //TODO get cap value
}