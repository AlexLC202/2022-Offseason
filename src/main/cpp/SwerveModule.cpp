#include "SwerveModule.h"

SwerveModule::SwerveModule(int turnID, int driveID, int cancoderID, double offset) : turnMotor_(turnID, "Drivebase"), driveMotor_(driveID, "Drivebase"), cancoder_(cancoderID, "Drivebase"), offset_(offset)
{
    turnMotor_.SetInverted(TalonFXInvertType::CounterClockwise);
    driveMotor_.SetInverted(TalonFXInvertType::Clockwise);

    turnMotor_.SetNeutralMode(NeutralMode::Brake);
    driveMotor_.SetNeutralMode(NeutralMode::Brake);

    id_ = std::to_string(driveID);

    cancoder_.ClearStickyFaults();//TODO check what this actually does
}

void SwerveModule::periodic(double driveSpeed, double angle)
{
    move(driveSpeed, angle);
}

void SwerveModule::move(double driveSpeed, double angle)
{
    //frc::SmartDashboard::PutNumber(id_ + " Wanted speed", driveSpeed);
    //frc::SmartDashboard::PutNumber(id_ + " Wanted angle", angle);

    units::volt_t turnVolts{calcAngPID(angle)};
    turnMotor_.SetVoltage(turnVolts);
    //frc::SmartDashboard::PutNumber(id_ + " Turn volts", turnVolts.value());

    //driveMotor_.Set(ControlMode::Velocity, driveSpeed * GeneralConstants::MAX_RPM);

    units::volt_t driveVolts{direction_ * calcDrivePID(driveSpeed)};
    driveMotor_.SetVoltage(driveVolts);
    //frc::SmartDashboard::PutNumber(id_ + " Drive volts", driveVolts.value());
    
}

double SwerveModule::calcAngPID(double setAngle)
{

    double error = findError(setAngle);
    //frc::SmartDashboard::PutNumber(id_ + "Angle error", error);

    aIntegralError_ += error * GeneralConstants::Kdt;
    double deltaError = (error - aPrevError_) / GeneralConstants::Kdt;
    if(abs(aPrevError_) < 2 && error > 5) //TODO get value
    {
        deltaError = 0;
        aIntegralError_ = 0;
    } 
    aPrevError_ = error;

    double power = (akP_*error) + (akI_*aIntegralError_) + (akD_*deltaError); //TODO implement integral anti-windup or just don't use kI

    //return 1.5;
    return std::clamp(power, -(double)GeneralConstants::MAX_VOLTAGE, (double)GeneralConstants::MAX_VOLTAGE);
}

/*void SwerveModule::normalizeAngle() //test later for math mistake
{
    //angle_ = turnMotor_.GetSelectedSensorPosition() * GEAR_RATIO * 360.0 / 2048.0;
    angle_ = cancoder_.GetAbsolutePosition();

    //angle_ += (360 * 100);

    //double angleDec = angle_ - floor(angle_);

    //angle_ = ((int)floor(angle_) % 360);
    //angle_ += angleDec;

    angle_ -= 360 * floor(angle_ / 360 + 0.5); 

}*/

/*double rpmToTicksPerDS(double rpm)
{
        return (rpm * TICKS_PER_ROTATION) / (600);
}*/

double SwerveModule::calcDrivePID(double driveSpeed)
{
    double velocity = (driveSpeed * GeneralConstants::MAX_RPM * GeneralConstants::TICKS_PER_ROTATION) / 600;
    double error = velocity - driveMotor_.GetSelectedSensorVelocity();

    //frc::SmartDashboard::PutNumber(id_ + "velocity error", error);

    dIntegralError_ += error * GeneralConstants::Kdt;
    double deltaError = (error - dPrevError_) / GeneralConstants::Kdt;
    dPrevError_ = error;

    if(abs(deltaError) < 40 && error > 100) //TODO get value, also test if needed
    {
        deltaError = 0;
        dIntegralError_ = 0;
    }

    double radPSec = (driveSpeed * GeneralConstants::MAX_RPM) * 2 * M_PI / 60;
    double feedForward = radPSec / GeneralConstants::Kv;
    //feedForward = 0;

    double power = (dkP_*error) + (dkI_*aIntegralError_) + (dkD_*deltaError) + feedForward;

    return std::clamp(power, -(double)GeneralConstants::MAX_VOLTAGE, (double)GeneralConstants::MAX_VOLTAGE);

}

double SwerveModule::findError(double setAngle)
{
    double rawError = setAngle - getAngle();

    //frc::SmartDashboard::PutNumber(id_ + "Angle", getAngle());

    Helpers::normalizeAngle(rawError);

    if(abs(rawError) > 90)
    {
        direction_ = -1;
    }
    else
    {
        direction_ = 1;
    }
    
    return (abs(rawError) <= 90) ? rawError : (rawError > 0) ? rawError - 180 : rawError + 180;
    /*if(abs(error) <= 90)
    {
        return error;
    }
    else
    {
        return (error > 0) ? error - 180 : error + 180;
    }*/

    //return rawError;

    

    /*double error = setAngle - getAngle();
    frc::SmartDashboard::PutNumber(id_ + "Angle", getAngle());*/

    //return error;
}

double SwerveModule::getDriveVelocity()
{
    return (driveMotor_.GetSelectedSensorVelocity() / 2048) * 10 * SwerveConstants::DRIVE_GEAR_RATIO * 2 * M_PI * SwerveConstants::TREAD_RADIUS;
}

double SwerveModule::getAngle()
{
    double angle = cancoder_.GetAbsolutePosition() + offset_;
    Helpers::normalizeAngle(angle);

    return angle;
}