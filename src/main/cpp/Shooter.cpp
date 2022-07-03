#include "Shooter.h"

Shooter::Shooter(Limelight* limelight, SwerveDrive* swerveDrive) : limelight_(limelight), swerveDrive_(swerveDrive), flywheelMaster_(ShooterConstants::FLYWHEEL_MASTER_ID), flywheelSlave_(ShooterConstants::FLYWHEEL_SLAVE_ID), kickerMotor_(ShooterConstants::KICKER_ID), flyTrajectoryCalc_(maxV, maxA, kP, kD, kV, kA), turret_(limelight_, swerveDrive_)
{
    flywheelMaster_.SetInverted(TalonFXInvertType::Clockwise);
    flywheelSlave_.Follow(flywheelMaster_);
    flywheelSlave_.SetInverted(InvertType::OpposeMaster);

    rangeAdjustment_ = 0;
    state_ = IDLE;
    createMap();
}

void Shooter::createMap()
{
    if(hasMap_)
    {
        return;
    }
    ifstream infile(ShooterConstants::SHOTS_FILE_NAME);
    
    cout << "started creating a map" << endl;

    string data;
    double distance, angle, velocity, partDer;

    bool valid;
    std::size_t c1, c2, c3;

    while(getline(infile, data))
    {
        valid = true;

        c1 = data.find(", ");
        if(c1 != string::npos)
        {
            distance = stod(data.substr(0, c1));

            c2 = data.find(", ", c1 + 1);
            if(c2 != string::npos)
            {
                angle = stod(data.substr(c1 + 2, c2));

                c3 = data.find(", ", c2 + 1);
                if(c3 != string::npos)
                {
                    velocity = stod(data.substr(c2 + 2, c3));
                    partDer = stod(data.substr(c3 + 2));
                }
                else
                {
                    valid = false;
                }
                
            }
            else
            {
                valid = false;
            }
        }
        else
        {
            valid = false;
        }

        if(valid)
        {
            tuple<double, double, double> shotData(angle, velocity, partDer);
            pair<double, tuple<double, double, double>> distancePoint(distance, shotData);

            shotsMap_.insert(distancePoint);
            ++mapPoints_;
            cout << "added a point" << endl;
        }
    }

    hasMap_ = true;
    infile.close();
}

Shooter::State Shooter::getState()
{
    return state_;
}

void Shooter::setState(State state)
{
    state_ = state;
}

void Shooter::setPID(double p, double i, double d)
{
    fKp_ = p;
    fKi_ = i;
    fKd_ = d;
}

void Shooter::setHoodPID(double p, double i, double d)
{
    hood_.setPID(p, i, d);
}

void Shooter::dewindIntegral()
{
    integralError_ = 0;
}

void Shooter::increaseRange()
{
    rangeAdjustment_ += ShooterConstants::Kr;
}

void Shooter::decreaseRange()
{
    rangeAdjustment_ -= ShooterConstants::Kr;
}

/*void Shooter::setColor(Channel::Color color)
{
    channel_.setColor(color);
}*/

void Shooter::setTurretManualVolts(double manualVolts)
{
    turret_.setManualVolts(manualVolts);
}

double Shooter::getHoodTicks()
{
    return hood_.getHoodTicks();
}

double Shooter::getTurretAngle()
{
    return turret_.getAngle();
}

double Shooter::getFlyPos()
{
    return flywheelMaster_.GetSelectedSensorPosition();
}

double Shooter::getFlyVel()
{
    return flywheelMaster_.GetSelectedSensorVelocity();
}

void Shooter::periodic(double yaw)
{
    if(state_ == UNLOADING)
    {
        limelight_->lightOn(false);
    }
    else
    {
        unloadShooting_ = false;
        limelight_->lightOn(true);
    }

    yaw_ = yaw;
    //swerveDrive->resetGoalOdometry(turret_.getAngle());
    swerveDrive_->calcOdometry(turret_.getAngle());

    //frc::SmartDashboard::PutBoolean("map", hasMap_);

    double hoodAngle, velocity, turretOffset, partDer, distance;
    //frc::SmartDashboard::PutBoolean("found target", swerveDrive->foundGoal());

    if((distance = limelight_->calcDistance()) != -1)
    {
        distance += rangeAdjustment_;
    }
    else if(swerveDrive_->foundGoal())
    {
        distance = rangeAdjustment_ + sqrt(swerveDrive_->getX() * swerveDrive_->getX() + swerveDrive_->getY() + swerveDrive_->getY());
    }
    else
    {
        distance = 0;
        hasShot_ = false;
    }
    
    frc::SmartDashboard::PutNumber("Range Adjustment", rangeAdjustment_);
    frc::SmartDashboard::PutNumber("Distance", distance);
    auto shot = shotsMap_.upper_bound(distance);
    if (shot != shotsMap_.begin() && shot != shotsMap_.end()) //TOOD test with distance?
    {
        //TODO disable interpolation when not using Andrew's points
        double higher = shot->first;
        double highHood = get<0>(shot->second);
        double highVel = get<1>(shot->second);

        --shot;
        partDer = get<2>(shot->second);

        swerveDrive_->getGoalXVel();
        swerveDrive_->getGoalYVel();
        /*tuple<double, double, double> shotVals = calcShootingWhileMoving(get<0>(shot->second), get<1>(shot->second), swerveDrive->getGoalXVel(), swerveDrive->getGoalYVel());
        hoodAngle = get<0>(shotVals);
        velocity = get<1>(shotVals);
        turretOffset = get<2>(shotVals);*/

        hoodAngle = get<0>(shot->second);
        velocity = get<1>(shot->second);
        turretOffset = 0;

        double skew = (distance - shot->first) / (higher - shot->first);

        hoodAngle += skew * (highHood - hoodAngle);
        velocity += skew * (highVel - velocity);

        hasShot_ = true;

        //frc::SmartDashboard::PutNumber("MAng", hoodAngle);
        //frc::SmartDashboard::PutNumber("MVel", velocity);
        //frc::SmartDashboard::PutNumber("MTOff", turretOffset);
    }
    else
    {
        velocity = 0;
        hoodAngle = ShooterConstants::MAX_HOOD_ANGLE;
        //hoodAngle = (ShooterConstants::MIN_HOOD_ANGLE + ShooterConstants::MAX_HOOD_ANGLE) / 2;
        turretOffset = 0;
        partDer = 1;
        hasShot_ = false;
    }

    //frc::SmartDashboard::PutNumber("V", flywheelMaster_.GetSelectedSensorVelocity() * 20 * M_PI * ShooterConstants::FLYWHEEL_RADIUS / (GeneralConstants::TICKS_PER_ROTATION * ShooterConstants::FLYWHEEL_GEAR_RATIO));

    if(partDer > 0.5)
    {
        hasShot_ = false; //TODO set value, see how auto shoot works?
    }

    if(hoodAngle > 0 || hoodAngle < ShooterConstants::MAX_HOOD_TICKS)
    {
        hoodAngle = 0;
        hasShot_ = false;
    }

    if(velocity < 0)
    {
        velocity = 0;
        hasShot_ = false;
    }

    if(abs(turretOffset) > 90)
    {
        std::cout << "Bro wtf is even happening with your math" << std::endl;
        turretOffset = 0;
        hasShot_ = false;
    }

    frc::SmartDashboard::PutBoolean("Hood Ready", hood_.isReady());
    frc::SmartDashboard::PutBoolean("Flywheel Ready", flywheelReady_);
    frc::SmartDashboard::PutBoolean("Turret Ready", turret_.isAimed());
    //frc::SmartDashboard::PutBoolean("Has Shot", hasShot_);

    //hasShot_ = true;
    shotReady_ = (flywheelReady_ && hood_.isReady() && turret_.isAimed() && hasShot_); //TODO something with have ball?

    //frc::SmartDashboard::PutBoolean("badIdea", channel_.badIdea());
    
    //TODO remove, testing
    //velocity = frc::SmartDashboard::GetNumber("InV", 0);
    //velocity = std::clamp(velocity, 0.0, ShooterConstants::MAX_VELOCITY);
    //hoodAngle = frc::SmartDashboard::GetNumber("InA", 0);
    //hoodAngle = std::clamp(hoodAngle, (double)ShooterConstants::MAX_HOOD_TICKS, 0.0);
    //turretOffset = 0;
    frc::SmartDashboard::PutBoolean("UNLOADING", (state_ == UNLOADING));

    switch(state_)
    {
        case IDLE:
        {
            hood_.setState(Hood::IDLE);
            turret_.setState(Turret::IDLE);
            break;
        }
        case TRACKING:
        {
            hood_.setWantedPos(hoodAngle);
            hood_.setState(Hood::AIMING);

            turret_.setState(Turret::TRACKING);
            //turret_.setState(Turret::MANUAL);

            flywheelMaster_.SetVoltage(units::volt_t (0));
            kickerMotor_.SetVoltage(units::volt_t(0));
            dewindIntegral();
            break;
        }
        case REVING: //TODO combine for auto shoot later, hood anti-windup
        {
            hood_.setWantedPos(hoodAngle);
            //hood_.setWantedPos(-1000);
            hood_.setState(Hood::AIMING);

            turret_.setState(Turret::TRACKING);
            //turret_.setState(Turret::MANUAL);

            //cout << velocity << endl;
            units::volt_t volts {calcFlyPID(velocity)};
            //units::volt_t volts {calcFlyPID(5000)};
            //units::volt_t volts{frc::SmartDashboard::GetNumber("FINV", 0)};

            //units::volt_t volts{calcFlyVolts(velocity)};
            flywheelMaster_.SetVoltage(volts);
            

            //flywheelMaster_.SetVoltage(units::volt_t(6));

            if(shotReady_)
            {
                //frc::SmartDashboard::PutBoolean("Shooting", true);
                kickerMotor_.SetVoltage(units::volt_t (4)); //TODO tune value
            }
            else
            {
                //frc::SmartDashboard::PutBoolean("Shooting", false);
                kickerMotor_.SetVoltage(units::volt_t(0));
            }
            break;
        }
        case UNLOADING:
        {
            hood_.setWantedPos(-2000);
            hood_.setState(Hood::AIMING);

            frc::SmartDashboard::PutNumber("FVEL", flywheelMaster_.GetSelectedSensorVelocity());
            frc::SmartDashboard::PutNumber("FCUR", flywheelMaster_.GetSupplyCurrent());
            turret_.setState(Turret::UNLOADING);
            //turret_.setState(Turret::MANUAL);

            units::volt_t volts {calcFlyPID(7000)}; //TODO get value or make a distance map
            flywheelMaster_.SetVoltage(volts);

            if(turret_.unloadReady() && flywheelEjectReady_)
            {
                kickerMotor_.SetVoltage(units::volt_t (4));
                if(flywheelMaster_.GetSupplyCurrent() > 20) //TODO get number
                {
                    unloadShooting_ = true;
                    cout << "current" << endl;
                }

                if(unloadShooting_ && flywheelMaster_.GetSupplyCurrent() < 20)
                {
                    state_ = TRACKING;
                    cout << "Stopped unloading" << endl;
                    frc::SmartDashboard::PutBoolean("Current", true);
                }
                
                /*if(flywheelMaster_.GetSelectedSensorVelocity() - prevVelocity_ < -10000) //TODO get number, test which is better (or both)
                {
                    state_ = TRACKING;
                    cout << "vel drop: " <<  endl;
                }*/

            }
            else
            {
                kickerMotor_.SetVoltage(units::volt_t(0));
            }
            break;
        }
        case MANUAL:
        {
            turret_.setState(Turret::MANUAL);
            hood_.setState(Hood::IDLE);

            flywheelMaster_.SetVoltage(units::volt_t(0));
            break;
        }
    }

    hood_.periodic();
    turret_.periodic(yaw_, turretOffset);
}

void Shooter::reset()
{
    hood_.reset();
    turret_.reset();
    rangeAdjustment_ = 0;
}

double Shooter::linVelToSensVel(double velocity)
{
    return ShooterConstants::FLYWHEEL_GEAR_RATIO * (velocity / ShooterConstants::FLYWHEEL_RADIUS) * GeneralConstants::TICKS_PER_ROTATION / ( 20 * M_PI);
}

double Shooter::calcFlyPID(double velocity)
{
    double time = timer_.GetFPGATimestamp().value();
    dT_ = time - prevTime_;
    prevTime_ = time;

    //double setAngVel = linVelToSensVel(velocity);
    //double error = setAngVel - flywheelMaster_.GetSelectedSensorVelocity();
    double error = velocity - flywheelMaster_.GetSelectedSensorVelocity();
    //frc::SmartDashboard::PutNumber("setFV", setAngVel);
    //frc::SmartDashboard::PutNumber("Ferror", error);
    //frc::SmartDashboard::PutNumber("FV", flywheelMaster_.GetSelectedSensorVelocity());

    integralError_ += error * dT_;
    double deltaError = (error - prevError_) / dT_;
    //cout << deltaError << endl;

    flywheelReady_ = (abs(error) < ShooterConstants::FLYWHEEL_READY/* && deltaError > -300 && abs(deltaError) < 500*/); //TODO get value
    flywheelEjectReady_ = (abs(error) < ShooterConstants::FLYWHEEL_EJECT_READY);
    if(abs(prevError_) < 40 && error > 100) //TODO get value, probably same as above
    {
        deltaError = 0;
        //integralError_ = 0;
    }
    prevError_ = error;
    prevVelocity_ = flywheelMaster_.GetSelectedSensorVelocity();

    //double radPSec = ((setAngVel / GeneralConstants::TICKS_PER_ROTATION) * 10 * 2 * M_PI);
    //double feedForward = radPSec / GeneralConstants::Kv;
    double feedForward = velocity / ShooterConstants::FLYWHEEL_FF;
    //frc::SmartDashboard::PutNumber("FF", feedForward);

    double power = (fKp_ * error) + (fKi_ * integralError_) + (fKd_ * deltaError) + feedForward;

    return std::clamp(power, -(double)GeneralConstants::MAX_VOLTAGE, (double)GeneralConstants::MAX_VOLTAGE);
}

double Shooter::calcFlyVolts(double velocity)
{
    double volts;
    double error = velocity - flywheelMaster_.GetSelectedSensorVelocity();

    flywheelReady_ = (abs(error) < ShooterConstants::FLYWHEEL_READY);
    flywheelEjectReady_ = (abs(error) < ShooterConstants::FLYWHEEL_EJECT_READY);
    frc::SmartDashboard::PutNumber("FError", error);

    if(abs(velocity - setTrajectoryVel_) > 100 && initTrajectory_) //TODO get value
    {
        setTrajectoryVel_ = velocity;
        double vel = flyTrajectoryCalc_.getVelProfile().second;
        flyTrajectoryCalc_.generateVelTrajectory(setTrajectoryVel_, vel);
    }
    if(!initTrajectory_)
    {
        initTrajectory_ = true;
        double vel = flywheelMaster_.GetSelectedSensorVelocity();
        flyTrajectoryCalc_.generateVelTrajectory(velocity, vel);
    }

    if(initTrajectory_)
    {
        double vel = flywheelMaster_.GetSelectedSensorVelocity();
        volts = flyTrajectoryCalc_.calcVelPower(vel);
    }
    else
    {
        volts = 0;
    }

    frc::SmartDashboard::PutNumber("FVEL", flywheelMaster_.GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber("FVOLTS", volts);
    frc::SmartDashboard::PutNumber("WVEL", flyTrajectoryCalc_.getVelProfile().second);

    return volts;
}

tuple<double, double, double> Shooter::calcShootingWhileMoving(double hoodAngle, double velocity, double goalXVel, double goalYVel)
{
    double zVel = velocity * sin(hoodAngle * M_PI / 180);
    double yVel = velocity * cos(hoodAngle * M_PI / 180) - goalYVel;
    double xVel = -goalXVel;

    double xyVel = sqrt(xVel * xVel + yVel * yVel);
    double newVel = sqrt(xyVel * xyVel + zVel * zVel);

    double turretAngle, newHoodAngle;
    if(yVel == 0 && xVel == 0)
    {
        turretAngle = 0;
    }
    else
    {
        turretAngle = -(atan2(yVel, xVel) * 180 / M_PI) + 90;
    }
    if(zVel == 0 && xyVel == 0)
    {
        newHoodAngle = ShooterConstants::MAX_HOOD_ANGLE;
        hasShot_ = false;
    }
    else
    {
        newHoodAngle = atan2(zVel, xyVel) * 180 / M_PI;
    }

    return tuple<double, double, double> (newHoodAngle, newVel, turretAngle);

}