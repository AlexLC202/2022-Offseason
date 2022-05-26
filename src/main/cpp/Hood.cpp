#include "Hood.h"

Hood::Hood() : hoodMotor_(ShooterConstants::HOOD_ID)
{
    hoodMotor_.SetInverted(true); //TODO test
    hoodMotor_.SetNeutralMode(NeutralMode::Brake);
    hoodMotor_.SetSelectedSensorPosition(0);

    state_ = IDLE;
}

Hood::State Hood::getState()
{
    return state_;
}

void Hood::setState(State state)
{
    state_ = state;
}

bool Hood::isReady()
{
    return atPos_;
}

void Hood::periodic()
{
    switch(state_)
    {
        case IDLE:
        {
            hoodMotor_.SetVoltage(units::volt_t(0));
            break;
        }
        case AIMING:
        {
            move();
            break;
        }
        case ZEROING:
        {
            zero();
            break;
        }
    }
}

void Hood::zero()
{
    //TODO yeah, zeroed bool?
}

void Hood::setWantedPos(double setPos)
{
    setPos_ = angleToTicks(setPos);
}

void Hood::move()
{
    units::volt_t volts{calcPID()};
    hoodMotor_.SetVoltage(volts);
}

double Hood::calcPID()
{
    frc::SmartDashboard::PutNumber("Angle", (hoodMotor_.GetSelectedSensorPosition() / ShooterConstants::TICKS_PER_DEGREE) + ShooterConstants::MIN_ANGLE);

    double error = setPos_ - hoodMotor_.GetSelectedSensorPosition();
    atPos_ = (abs(error) < 0.01); //TODO get value

    integralError_ += error * GeneralConstants::Kdt;
    double deltaError = (error - prevError_) / GeneralConstants::Kdt;
    prevError_ = error;

    double power = (kP_*error) + (kI_*integralError_) + (kD_*deltaError);

    return std::clamp(power, -(double)GeneralConstants::MAX_VOLTAGE, (double)GeneralConstants::MAX_VOLTAGE);
}

double Hood::angleToTicks(double angle)
{
    return (angle - ShooterConstants::MIN_ANGLE) * ShooterConstants::TICKS_PER_DEGREE;
}
