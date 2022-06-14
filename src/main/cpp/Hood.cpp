#include "Hood.h"

Hood::Hood() : hoodMotor_(ShooterConstants::HOOD_ID)
{
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

void Hood::setPID(double p, double i, double d)
{
    kP_ = p;
    kI_ = i;
    kD_ = d;
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
    hoodMotor_.SetVoltage(units::volt_t(1));

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
    double volts = calcPID();
    if(hoodMotor_.GetSelectedSensorPosition() < ShooterConstants::MAX_HOOD_TICKS && volts < 0)
    {
        hoodMotor_.SetVoltage(units::volt_t(0));
    }
    else if(hoodMotor_.GetSelectedSensorPosition() > 0 && volts > 0)
    {
        hoodMotor_.SetVoltage(units::volt_t(0));
    }
    else
    {
        hoodMotor_.SetVoltage(units::volt_t(volts));
    }
    
}

double Hood::calcPID()
{
    frc::SmartDashboard::PutNumber("Ang", (hoodMotor_.GetSelectedSensorPosition() / ShooterConstants::TICKS_PER_HOOD_DEGREE) + ShooterConstants::MAX_HOOD_ANGLE);
    //frc::SmartDashboard::PutNumber("Ang Ticks", hoodMotor_.GetSelectedSensorPosition());

    double error = setPos_ - hoodMotor_.GetSelectedSensorPosition();

    integralError_ += error * GeneralConstants::Kdt;
    double deltaError = (error - prevError_) / GeneralConstants::Kdt;
    if(abs(prevError_) < 50.0 && abs(error > 125)) //TODO get value, probably same as above
    {
        deltaError = 0;
        integralError_ = 0;
    } 
    prevError_ = error;

    atPos_ = (abs(error) < 100.0 && abs(deltaError) < 200); //TODO get value, change back to 50

    double power = (kP_*error) + (kI_*integralError_) + (kD_*deltaError);
    power += ShooterConstants::HOOD_FF;

    frc::SmartDashboard::PutNumber("HE", error);
    //frc::SmartDashboard::PutNumber("HP", power);
    return std::clamp(power, -(double)GeneralConstants::MAX_VOLTAGE * 0.3, (double)GeneralConstants::MAX_VOLTAGE * 0.3);
}

double Hood::angleToTicks(double angle)
{
    return (angle - ShooterConstants::MAX_HOOD_ANGLE) * ShooterConstants::TICKS_PER_HOOD_DEGREE;
}
