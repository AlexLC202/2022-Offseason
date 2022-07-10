#include "Turret.h"

Turret::Turret(Limelight* limelight, SwerveDrive* swerveDrive) : turretMotor_(ShooterConstants::TURRET_ID), trajectoryCalc_(maxV, maxA, kP, kD, kV, kA)
{
    turretMotor_.SetNeutralMode(NeutralMode::Coast);
    reset();
    limelight_ = limelight;
    swerveDrive_ = swerveDrive;

    state_ = IDLE;
}

void Turret::periodic(double yaw, double offset)
{
    yaw_ = yaw;
    offset_ = offset;
    //robotGoalAng_ = robotGoalAng;
    //foundGoal_ = foundGoal;
    //x_ = x;
    //y_ = y;

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
            //frc::SmartDashboard::PutNumber("TPOS", getAngle());
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
    //return true;
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
    initTrajectory_ = false;
}

void Turret::track()
{
    if(!limelight_->hasTarget() && !swerveDrive_->foundGoal())
    {
        turretMotor_.SetVoltage(units::volt_t(0));
    }
    else
    {
        double volts = calcPID();

        /*double volts;
        double error = calcError();

        //double inPos = frc::SmartDashboard::GetNumber("InT", getAngle());
        //Helpers::normalizeAngle(setPos);
        //error = inPos - getAngle();

        double pos = getAngle();
        double setPos = pos + error;
        double vel = turretMotor_.GetSelectedSensorVelocity() * 10 / ShooterConstants::TICKS_PER_TURRET_DEGREE;
        if(initTrajectory_)
        {
            pos = get<2>(trajectoryCalc_.getProfile());
            vel = get<1>(trajectoryCalc_.getProfile());
            trajectoryCalc_.generateTrajectory(pos, setPos, vel);

            volts = trajectoryCalc_.calcPower(pos, vel);
        }
        else
        {
            initTrajectory_ = true;
            volts = 0;
            trajectoryCalc_.generateTrajectory(pos, setPos, vel);
            volts = trajectoryCalc_.calcPower(pos, vel);
        }*/


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
            turretMotor_.SetVoltage(units::volt_t(volts));
        }
        
    }
    
}

void Turret::calcUnloadAng()
{
    double angToHangar = 0;
    double x = swerveDrive_->getX();
    double y = swerveDrive_->getY();

    double xDist = x - GeneralConstants::HANGAR_X;
    double yDist = y - GeneralConstants::HANGAR_Y;
    if(xDist != 0 || yDist != 0)
    {
        angToHangar = -(atan2(yDist, xDist) * 180 / M_PI) - 90;
    }

    double angToGoal = 0;
    if(x != 0 || y != 0)
    {
        angToGoal = -(atan2(y, x) * 180 / M_PI) - 90;
    }

    angToHangar += 360 * 10;
    angToHangar = ((int)floor(angToHangar) % 360) + (angToHangar - floor(angToHangar));

    angToGoal += 360 * 10; 
    angToGoal = ((int)floor(angToGoal) % 360) + (angToGoal - floor(angToGoal));

    if(abs(angToHangar - angToGoal) < 10) //TODO get value
    {
        angToHangar += (angToHangar > angToGoal) ? 10 : -10; //TODO incorporate goal diameter?
    }
    //Helpers::normalizeAngle(angToHangar);

    unloadAngle_ = 180 + angToHangar - yaw_;
    Helpers::normalizeAngle(unloadAngle_);
}

double Turret::calcAngularFF()
{
    double deltaYaw = yaw_ - prevYaw_;
    frc::SmartDashboard::PutNumber("PYAW", prevYaw_);
    prevYaw_ = yaw_;

    if(abs(deltaYaw) > 300)
    {
        deltaYaw = (deltaYaw > 0) ? deltaYaw - 360 : deltaYaw + 360;
    }

    double yawVel = deltaYaw / dT_;

    double radPerSec = -(yawVel * ShooterConstants::TICKS_PER_TURRET_DEGREE * 2 * M_PI) / GeneralConstants::TICKS_PER_ROTATION;
    
    double rff = radPerSec / GeneralConstants::Kv; //TODO tune

    double ff = yawVel / ShooterConstants::TURRET_FF;

    frc::SmartDashboard::PutNumber("YAW3", yaw_);
    
    frc::SmartDashboard::PutNumber("DYAW", deltaYaw);
    frc::SmartDashboard::PutNumber("TFF", ff);
    frc::SmartDashboard::PutNumber("{TFF", rff);
    //return 0;

    return rff;
}

double Turret::calcLinearFF()
{
    double vel = swerveDrive_->getGoalXVel();
    double x = swerveDrive_->getX();
    double y = swerveDrive_->getY();
    double distance = sqrt(x * x + y * y);

    //double radPerSec = (-vel / distance) * 360 * ShooterConstants::TICKS_PER_TURRET_DEGREE / GeneralConstants::TICKS_PER_ROTATION;

    double degPerSec = (-vel / distance) * 180 / M_PI;

    return degPerSec / ShooterConstants::TURRET_FF;
    //return radPerSec / GeneralConstants::Kv; //TODO tune
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
        double wantedTurretAng = (180 - swerveDrive_->getRobotGoalAng()) + offset_;
        Helpers::normalizeAngle(wantedTurretAng);

        error =  wantedTurretAng - getAngle();
    }

    //frc::SmartDashboard::PutNumber("TA", getAngle());
    frc::SmartDashboard::PutNumber("Terror", error);

    if(abs(error + getAngle()) > 180)
    {
        error = (error > 0) ? error - 360 : error + 360;
    }

    if(abs(error) > 60) //COMP disable probably
    {
        //limelight_->lightOn(false);
    }
    else
    {
        limelight_->lightOn(true);
    }

    aimed_ = (abs(error) < ShooterConstants::TURRET_AIMED); //TODO get value, change back to 2.5
    unloadReady_ = (abs(error) < ShooterConstants::TURRET_UNLOAD_AIMED); //TODO get value

    return error;
}

double Turret::calcPID()
{
    double time = timer_.GetFPGATimestamp().value();
    dT_ = time - prevTime_;
    prevTime_ = time;

    double error = calcError();
    
    double deltaError = (error - prevError_) / dT_;
    integralError_ += error * dT_;

    if(abs(prevError_) < 2.5 && abs(error > 5)) //TODO get value, probably same as above
    {
        deltaError = 0;
        integralError_ = 0;
    } 
    prevError_ = error;

    double power = (tkP_*error) + (tkI_*integralError_) + (tkD_*deltaError);
    power += calcAngularFF();
    //power += calcLinearFF();
    frc::SmartDashboard::PutNumber("LTFF", calcLinearFF());

    //TODO, disable voltage limit for ffs... like what?

    return std::clamp(power, -(double)GeneralConstants::MAX_VOLTAGE * 0.3, (double)GeneralConstants::MAX_VOLTAGE * 0.3); //TODO get cap value
}