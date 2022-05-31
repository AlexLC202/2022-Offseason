#include "Hood.h"

Hood::Hood() : hoodMotor_(ShooterConstants::HOOD_ID)
{
    hoodMotor_.SetInverted(TalonFXInvertType::CounterClockwise); //TODO test
    hoodMotor_.SetNeutralMode(NeutralMode::Coast);
    reset();

    state_ = IDLE;
}

Hood::State Hood::getState()
{
    return state_;
}

void Hood::setState(State state)
{
    if(state == ZEROING)
    {
        zeroed_ = false;
    }
    state_ = state;
}

bool Hood::isReady()
{
    return atPos_;
}

void Hood::periodic()
{
    if(!zeroed_)
    {
        state_ = ZEROING;
    }

    switch(state_)
    {
        case IDLE:
        {
            hoodMotor_.SetNeutralMode(NeutralMode::Coast);
            hoodMotor_.SetVoltage(units::volt_t(0));
            break;
        }
        case IMMOBILE:
        {
            hoodMotor_.SetNeutralMode(NeutralMode::Brake);
            hoodMotor_.SetVoltage(units::volt_t(0));
            break;
        }
        case AIMING:
        {
            hoodMotor_.SetNeutralMode(NeutralMode::Brake);
            move();
            break;
        }
        case ZEROING:
        {
            hoodMotor_.SetNeutralMode(NeutralMode::Brake);
            zero();
            break;
        }
    }
}

void Hood::reset()
{
    zeroed_ = true;
    hoodMotor_.SetSelectedSensorPosition(0);
}

void Hood::zero()
{
    hoodMotor_.SetVoltage(units::volt_t(-1)); //TODO test direction

    if(hoodMotor_.GetSupplyCurrent() > ShooterConstants::HOOD_ZERO_CURRENT)
    {
        hoodMotor_.SetVoltage(units::volt_t(0));
        reset();
        state_ = IMMOBILE;
    }
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
    frc::SmartDashboard::PutNumber("Angle", (hoodMotor_.GetSelectedSensorPosition() / ShooterConstants::TICKS_PER_HOOD_DEGREE) + ShooterConstants::MIN_HOOD_ANGLE);

    double error = setPos_ - hoodMotor_.GetSelectedSensorPosition();
    atPos_ = (abs(error) < 50.0); //TODO get value
    //std::cout << error << std::endl;

    integralError_ += error * GeneralConstants::Kdt;
    double deltaError = (error - prevError_) / GeneralConstants::Kdt;
    prevError_ = error;

    double power = (kP_*error) + (kI_*integralError_) + (kD_*deltaError);

    return std::clamp(power, -(double)GeneralConstants::MAX_VOLTAGE, (double)GeneralConstants::MAX_VOLTAGE);
}

double Hood::angleToTicks(double angle)
{
    return (angle - ShooterConstants::MIN_HOOD_ANGLE) * ShooterConstants::TICKS_PER_HOOD_DEGREE;
}
