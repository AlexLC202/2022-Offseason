#include "Shooter.h"

Shooter::Shooter(Limelight* limelight, Channel* channel) : limelight_(limelight), channel_(channel), flywheelMaster_(ShooterConstants::FLYWHEEL_MASTER_ID), flywheelSlave_(ShooterConstants::FLYWHEEL_SLAVE_ID), kickerMotor_(ShooterConstants::KICKER_ID), turret_(limelight_)
{
    flywheelMaster_.SetInverted(TalonFXInvertType::Clockwise);
    flywheelSlave_.Follow(flywheelMaster_);
    flywheelSlave_.SetInverted(InvertType::OpposeMaster);

    state_ = IDLE;
    createMap();
}

void Shooter::createMap()
{
    if(hasMap_)
    {
        return;
    }
    ifstream infile(ShooterConstants::SHOTS_FILE_NAME); //src/main/deploy/shots.csv
    
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

void Shooter::setTurretManualVolts(double manualVolts)
{
    turret_.setManualVolts(manualVolts);
}

void Shooter::periodic(double yaw, SwerveDrive* swerveDrive)
{
    yaw_ = yaw;
    //swerveDrive->resetGoalOdometry(turret_.getAngle());
    swerveDrive->calcOdometry(turret_.getAngle());

    //frc::SmartDashboard::PutBoolean("map", hasMap_);

    double hoodAngle, velocity, turretOffset, partDer, distance;
    //frc::SmartDashboard::PutBoolean("found target", swerveDrive->foundGoal());

    if((distance = limelight_->calcDistance()) != -1)
    {
        distance += rangeAdjustment_;
    }
    else if(swerveDrive->foundGoal())
    {
        distance = rangeAdjustment_ + sqrt(swerveDrive->getX() * swerveDrive->getX() + swerveDrive->getY() + swerveDrive->getY());
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
        --shot;
        partDer = get<2>(shot->second);

        /*tuple<double, double, double> shotVals = calcShootingWhileMoving(get<0>(shot->second), get<1>(shot->second), swerveDrive->getRGoalXVel(), swerveDrive->getRGoalYVel());
        hoodAngle = get<0>(shotVals);
        velocity = get<1>(shotVals);
        turretOffset = get<2>(shotVals);*/

        hoodAngle = get<0>(shot->second);
        velocity = get<1>(shot->second);
        turretOffset = 0;

        hasShot_ = true;

        frc::SmartDashboard::PutNumber("MAng", hoodAngle);
        frc::SmartDashboard::PutNumber("MVel", velocity);
        frc::SmartDashboard::PutNumber("MTOff", turretOffset);
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

    //frc::SmartDashboard::PutNumber("NVel", velocity);
    //frc::SmartDashboard::PutNumber("NAng", angle);
    //frc::SmartDashboard::PutNumber("NTOff", turretOffset);

    //frc::SmartDashboard::PutNumber("V", flywheelMaster_.GetSelectedSensorVelocity() * 20 * M_PI * ShooterConstants::FLYWHEEL_RADIUS / (GeneralConstants::TICKS_PER_ROTATION * ShooterConstants::FLYWHEEL_GEAR_RATIO));

    if(partDer > 0.5)
    {
        hasShot_ = false; //TODO set value, see how auto shoot works?
    }

    if(hoodAngle > ShooterConstants::MAX_HOOD_ANGLE || hoodAngle < ShooterConstants::MIN_HOOD_ANGLE)
    {
        hoodAngle = ShooterConstants::MAX_HOOD_ANGLE;
        hasShot_ = false;
    }

    if(velocity < 0 || velocity > ShooterConstants::MAX_VELOCITY)
    {
        velocity = 0;
        hasShot_ = false;
    }

    if(turretOffset > 90 || turretOffset < -90)
    {
        std::cout << "Bro wtf is even happening with your math" << std::endl;
        turretOffset = 0;
        hasShot_ = false;
    }

    frc::SmartDashboard::PutBoolean("Hood Ready", hood_.isReady());
    frc::SmartDashboard::PutBoolean("Flywheel Ready", flywheelReady_);
    frc::SmartDashboard::PutBoolean("Turret Ready", turret_.isAimed());
    frc::SmartDashboard::PutBoolean("Has Shot", hasShot_);
    shotReady_ = (flywheelReady_ && hood_.isReady() && turret_.isAimed() && hasShot_); //TODO something with have ball?

    if(channel_->badIdea())
    {
        state_ = UNLOADING;
    }
    
    //TODO remove, testing
    velocity = frc::SmartDashboard::GetNumber("InV", 0);
    hoodAngle = frc::SmartDashboard::GetNumber("InA", ShooterConstants::MAX_HOOD_ANGLE);
    hoodAngle = std::clamp(hoodAngle, ShooterConstants::MIN_HOOD_ANGLE, ShooterConstants::MAX_HOOD_ANGLE);
    turretOffset = 0;

    switch(state_)
    {
        case IDLE:
        {
            hood_.setState(Hood::State::IDLE);
            turret_.setState(Turret::State::IDLE);
            break;
        }
        case TRACKING:
        {
            hood_.setWantedPos(hoodAngle);
            hood_.setState(Hood::State::AIMING);

            turret_.setState(Turret::State::TRACKING);

            flywheelMaster_.SetVoltage(units::volt_t (1));
            kickerMotor_.SetVoltage(units::volt_t(0));
            dewindIntegral();
            break;
        }
        case REVING: //TODO combine for auto shoot later, hood anti-windup
        {
            hood_.setWantedPos(hoodAngle);
            hood_.setState(Hood::State::AIMING);

            turret_.setState(Turret::State::TRACKING);

            units::volt_t volts {calcFlyPID(velocity)};
            flywheelMaster_.SetVoltage(volts);

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
            hood_.setWantedPos(ShooterConstants::MAX_HOOD_ANGLE - 5); //TODO get all values
            hood_.setState(Hood::State::AIMING);

            turret_.setState(Turret::State::UNLOADING);

            units::volt_t volts {calcFlyPID(6)}; //TODO get value or make a distance map
            flywheelMaster_.SetVoltage(volts);

            if(turret_.unloadReady() && flywheelReady_)
            {
                kickerMotor_.SetVoltage(units::volt_t (4));
            }
            else
            {
                kickerMotor_.SetVoltage(units::volt_t(0));
            }
            break;
        }
        case MANUAL:
        {
            turret_.setState(Turret::State::MANUAL);
            hood_.setState(Hood::State::IDLE);

            flywheelMaster_.SetVoltage(units::volt_t(0));
            break;
        }
    }

    hood_.periodic();
    turret_.periodic(yaw_, turretOffset, swerveDrive->getRobotGoalAng(), swerveDrive->foundGoal(), swerveDrive->getX(), swerveDrive->getY());
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
    //velocityAdjustment_ = frc::SmartDashboard::GetNumber("Velocity Adjustment", 0);
    //frc::SmartDashboard::PutNumber("Velocity Adjustment", velocityAdjustment_);

    double setAngVel = linVelToSensVel(velocity)/* + velocityAdjustment_*/; //TODO get value
    double error = setAngVel - flywheelMaster_.GetSelectedSensorVelocity();
    //frc::SmartDashboard::PutNumber("setFV", setAngVel);
    frc::SmartDashboard::PutNumber("Ferror", error);
    //frc::SmartDashboard::PutNumber("FV", flywheelMaster_.GetSelectedSensorVelocity());

    integralError_ += error * GeneralConstants::Kdt;
    double deltaError = (error - prevError_) / GeneralConstants::Kdt;
    cout << deltaError << endl;

    flywheelReady_ = (abs(error) < 40 && deltaError > -300 && abs(deltaError) < 500); //TODO get value
    flywheelEjectReady_ = (abs(error) < 100); //TODO get value
    if(abs(prevError_) < 40 && error > 100) //TODO get value, probably same as above
    {
        deltaError = 0;
        //integralError_ = 0;
    }
    prevError_ = error;

    double radPSec = ((setAngVel / GeneralConstants::TICKS_PER_ROTATION) * 10 * 2 * M_PI);
    double feedForward = radPSec / GeneralConstants::Kv;

    double power = (fKp_ * error) + (fKi_ * integralError_) + (fKd_ * deltaError) + feedForward;

    return std::clamp(power, -(double)GeneralConstants::MAX_VOLTAGE, (double)GeneralConstants::MAX_VOLTAGE);
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