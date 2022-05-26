#include "Shooter.h"

Shooter::Shooter() : flywheelMaster_(ShooterConstants::FLYWHEEL_MASTER_ID), flywheelSlave_(ShooterConstants::FLYWHEEL_SLAVE_ID), kickerMotor_(ShooterConstants::KICKER_ID)
{
    flywheelMaster_.SetInverted(TalonFXInvertType::CounterClockwise);
    flywheelSlave_.Follow(flywheelMaster_);
    flywheelSlave_.SetInverted(InvertType::OpposeMaster);
    state_ = IDLE;
    //TODO constructor stuff for others or this or idk my brain can't focus on anything but what's right in front of it right now and I'm kinda dying I want to become a potato
}

Shooter::State Shooter::getState()
{
    return state_;
}

void Shooter::setState(State state)
{
    state_ = state;
}

void Shooter::periodic()
{
    //TODO get distance with limelight, calc distance
    double distance = limelight_.calcDistance();
    double angle = ShooterConstants::MIN_ANGLE;
    double velocity = 0;


    angle = frc::SmartDashboard::GetNumber("Set angle", 0);
    frc::SmartDashboard::PutNumber("Set angle", angle);

    velocity = frc::SmartDashboard::GetNumber("Set velocity", 0);
    frc::SmartDashboard::PutNumber("Set velocity", velocity);

    frc::SmartDashboard::PutNumber("Velocity", flywheelMaster_.GetSelectedSensorVelocity() * 20 * M_PI * ShooterConstants::FLYWHEEL_RADIUS/ GeneralConstants::TICKS_PER_ROTATION);

    shotReady_ = (flywheelReady_ && hood_.isReady()); //TODO something with have ball?

    switch(state_)
    {
        case IDLE:
        {
            hood_.setState(Hood::State::IDLE);
            break;
        }
        case TRACKING:
        {
            flywheelMaster_.SetVoltage(units::volt_t (0));
            hood_.setWantedPos(angle);
            hood_.setState(Hood::State::AIMING);
            break;
        }
        case REVING: //TODO combine for auto shoot later, hood anti-windup
        {
            hood_.setWantedPos(angle);
            hood_.setState(Hood::State::AIMING);

            units::volt_t volts {calcFlyPID(velocity)};
            flywheelMaster_.SetVoltage(volts);

            if(shotReady_)
            {
                kickerMotor_.SetVoltage(units::volt_t (4)); //TODO tune value
            }
            else
            {
                kickerMotor_.SetVoltage(units::volt_t(0));
            }
            break;
        }
        case UNLOADING:
        {
            break;
        }
        case MANUAL:
        {
            break;
        }
    }
}

double Shooter::linVelToSensVel(double velocity)
{
    return ShooterConstants::FLYWHEEL_GEAR_RATIO * (velocity / ShooterConstants::FLYWHEEL_RADIUS) * GeneralConstants::TICKS_PER_ROTATION / ( 20 * M_PI);
}

double Shooter::calcFlyPID(double velocity)
{
    double setAngVel = linVelToSensVel(velocity);
    double error = setAngVel - flywheelMaster_.GetSelectedSensorVelocity();
    flywheelReady_ = (abs(error) < 300); //TODO get value

    integralError_ += error * GeneralConstants::Kdt;
    double deltaError = (error - prevError_) / GeneralConstants::Kdt;
    prevError_ = error;

    double radPSec = ((setAngVel / GeneralConstants::TICKS_PER_ROTATION) * 10 * 2 * M_PI);
    double feedForward = radPSec / GeneralConstants::Kv;

    double power = (fKp_ * error) + (fKi_ * integralError_) + (fKd_ * deltaError) + feedForward;

    frc::SmartDashboard::PutNumber("Flywheel volts", power);
    return std::clamp(power, -(double)GeneralConstants::MAX_VOLTAGE, (double)GeneralConstants::MAX_VOLTAGE);
}