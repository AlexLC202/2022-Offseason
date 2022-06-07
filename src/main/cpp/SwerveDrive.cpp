#include "SwerveDrive.h"

SwerveDrive::SwerveDrive(Limelight* limelight)
{
    limelight_ = limelight;
}

void SwerveDrive::periodic(double yaw, Controls* controls)
{
    yaw_ = yaw;
    drive(controls->getXStrafe(), controls->getYStrafe(), controls->getTurn());
}

void SwerveDrive::drive(double xSpeed, double ySpeed, double turn)
{
    /*double p = frc::SmartDashboard::GetNumber("p", 0);
    frc::SmartDashboard::PutNumber("p", p);
    topRight_->setP(p);
    topLeft_->setP(p);
    bottomRight_->setP(p);
    bottomLeft_->setP(p);

    double d = frc::SmartDashboard::GetNumber("d", 0);
    frc::SmartDashboard::PutNumber("d", d);
    topRight_->setD(d);
    topLeft_->setD(d);
    bottomRight_->setD(d);
    bottomLeft_->setD(d);*/

    calcOdometry();
    calcModules(xSpeed, ySpeed, turn);

    topRight_->periodic(trSpeed_, trAngle_);
    topLeft_->periodic(tlSpeed_, tlAngle_);
    bottomRight_->periodic(brSpeed_, brAngle_);
    bottomLeft_->periodic(blSpeed_, blAngle_);

}

void SwerveDrive::calcModules(double xSpeed, double ySpeed, double turn)
{
    double angle = yaw_ * M_PI / 180;

    double newX = xSpeed * cos(angle) + ySpeed * sin(angle);
    double newY = ySpeed * cos(angle) + xSpeed * -sin(angle);

    double A = newX - (turn);
    double B = newX + (turn);
    double C = newY - (turn);
    double D = newY + (turn);

    trSpeed_ = sqrt(B*B + C*C);
    tlSpeed_ = sqrt(B*B + D*D);
    brSpeed_ = sqrt(A*A + C*C);
    blSpeed_ = sqrt(A*A + D*D);

    if(xSpeed != 0 || ySpeed != 0 || turn != 0)
    {
        trAngle_ = -atan2(B, C) * 180 / M_PI;
        tlAngle_ = -atan2(B, D) * 180 / M_PI;
        brAngle_ = -atan2(A, C) * 180 / M_PI;
        blAngle_ = -atan2(A, D) * 180 / M_PI;
    }

    if(trSpeed_ > 1 || tlSpeed_ > 1 || brSpeed_ > 1 || brSpeed_ > 1)
    {
        double max = trSpeed_;

        max = (tlSpeed_ > max) ? tlSpeed_ : max;
        max = (brSpeed_ > max) ? brSpeed_ : max;
        max = (blSpeed_ > max) ? blSpeed_ : max;

        trSpeed_ = (trSpeed_ / max);
        tlSpeed_ = (tlSpeed_ / max);
        brSpeed_ = (brSpeed_ / max);
        blSpeed_ = (blSpeed_ / max);
    }
}

void SwerveDrive::calcOdometry()
{
    double angle = yaw_ * M_PI / 180;

    double goalAngle = (-yaw_ - yawOffset_);
    goalAngle += 360 * 10;
    goalAngle = ((int)floor(goalAngle) % 360) + (goalAngle - floor(goalAngle));
    goalAngle -= 360 * floor(goalAngle / 360 + 0.5);
    
    goalAngle = goalAngle * M_PI / 180;

    double frX = topRight_->getDriveVelocity() * sin(topRight_->getAngle());
    double frY = topRight_->getDriveVelocity() * cos(topRight_->getAngle());
    double flX = topLeft_->getDriveVelocity() * sin(topLeft_->getAngle());
    double flY = topLeft_->getDriveVelocity() * cos(topLeft_->getAngle());
    double brX = bottomRight_->getDriveVelocity() * sin(bottomRight_->getAngle());
    double brY = bottomRight_->getDriveVelocity() * cos(bottomRight_->getAngle());
    double blX = bottomLeft_->getDriveVelocity() * sin(bottomLeft_->getAngle());
    double blY = bottomLeft_->getDriveVelocity() * cos(bottomLeft_->getAngle());

    double avgX = (frX + flX + brX + blX) / 4;
    double avgY = (frY + flY + brY + blY) / 4;
    
    double rotatedX = avgX * cos(angle) + avgY * -sin(angle);
    double rotatedY = avgX * sin(angle) + avgY * cos(angle);

    goalXVel_ = avgX * cos(goalAngle) + avgY * sin(goalAngle);
    goalYVel_ = avgX * -sin(goalAngle) + avgY * cos(goalAngle);

    x_ += rotatedX * GeneralConstants::Kdt;
    y_ += rotatedY * GeneralConstants::Kdt;

    goalX_ += goalXVel_ * GeneralConstants::Kdt;
    goalY_ += goalYVel_ * GeneralConstants::Kdt;

    frc::SmartDashboard::PutNumber("x", x_);
    frc::SmartDashboard::PutNumber("y", y_);
    frc::SmartDashboard::PutNumber("gx", goalX_);
    frc::SmartDashboard::PutNumber("gy", goalY_);
}

/*void SwerveDrive::calcOdometry2()
{
    double trRotAng = SwerveConstants::trPosAngle - topRight_->getAngle(); //TODO check like everything, idk if this method even works
    double tlRotAng = SwerveConstants::tlPosAngle - topLeft_->getAngle();
    double brRotAng = SwerveConstants::brPosAngle - bottomRight_->getAngle();
    double blRotAng = SwerveConstants::blPosAngle - bottomLeft_->getAngle();

    double trVel = topRight_->getDriveVelocity();
    double tlVel = topLeft_->getDriveVelocity();
    double brVel = bottomRight_->getDriveVelocity();
    double blVel = topLeft_->getDriveVelocity();

    double trRotVel = sin(trRotAng * 180 / M_PI) * trVel; //TODO check sin vs cos
    double tlRotVel = sin(tlRotAng * 180 / M_PI) * tlVel;
    double brRotVel = sin(brRotAng * 180 / M_PI) * brVel;
    double blRotVel = sin(blRotAng * 180 / M_PI) * blVel;

    double trLinVel = cos(trRotAng * 180 / M_PI) * trVel;
    double tlLinVel = cos(tlRotAng * 180 / M_PI) * tlVel;
    double brLinVel = cos(brRotAng * 180 / M_PI) * brVel;
    double blLinVel = cos(blRotAng * 180 / M_PI) * blVel;

    double r = sqrt(SwerveConstants::LENGTH * SwerveConstants::LENGTH + SwerveConstants::WIDTH * SwerveConstants::WIDTH);
    double rotVel = (trRotVel + tlRotVel + brRotVel + blRotVel) / (4 * r); //TOOD divide by 4 or do some other werid stuff?

    double linVel = sqrt((trLinVel + blLinVel) * (trLinVel + blLinVel) + (tlLinVel + brLinVel) * (tlLinVel + brLinVel)); //TODO check math

    double linVelAng = atan2((trLinVel + blLinVel), (tlLinVel + brLinVel));//TODO UNFINISHED

    double trXVel = trLinVel * sin(SwerveConstants::trPosAngle);
    double tlXVel = tlLinVel * sin(SwerveConstants::tlPosAngle);
    double brXVel = brLinVel * sin(SwerveConstants::brPosAngle);
    double blXVel = blLinVel * sin(SwerveConstants::blPosAngle);
    
    double trYVel = trLinVel * cos(SwerveConstants::trPosAngle);
    double tlYVel = tlLinVel * cos(SwerveConstants::tlPosAngle);
    double brYVel = brLinVel * cos(SwerveConstants::brPosAngle);
    double blYVel = blLinVel * cos(SwerveConstants::blPosAngle);

    double xVel = (trXVel + tlXVel + brXVel + blXVel) / 4;
    double yVel = (trYVel + tlYVel + brYVel + blYVel) / 4;

}*/

void SwerveDrive::resetGoalOdometry(double turretAngle)
{
    if(!limelight_->hasTarget())
    {
        return;
    }

    foundGoal_ = true;
    goalY_ = - (limelight_->calcDistance() + 0.6096); //center of goal is origin
    goalX_ = 0;
    yawOffset_ = -yaw_ - (180 - (turretAngle + limelight_->getXOff()));
}

double SwerveDrive::getRobotGoalAng()
{
    if(foundGoal_)
    {
        
        double robotGoalAng = -yaw_ - yawOffset_;

        robotGoalAng += 360 * 10;
        robotGoalAng = ((int)floor(robotGoalAng) % 360) + (robotGoalAng - floor(robotGoalAng));
        robotGoalAng -= 360 * floor(robotGoalAng / 360 + 0.5);

        frc::SmartDashboard::PutNumber("rga", robotGoalAng);
        return robotGoalAng;
    }
    else
    {
        return 0;
    }
    
}

bool SwerveDrive::foundGoal()
{
    return foundGoal_;
}

void SwerveDrive::setFoundGoal(bool foundGoal)
{
    foundGoal_ = foundGoal;
}

double SwerveDrive::getGoalX()
{
    return goalX_;
}

double SwerveDrive::getGoalY()
{
    return goalY_;
}

double SwerveDrive::getRGoalXVel()
{
    if(limelight_->hasTarget())
    {
        return goalXVel_;
    }

    if(goalY_ == 0 && goalX_ ==0)
    {
        return 0;
    }
    double angToGoal = (atan2(-goalY_, -goalX_) * 180 / M_PI);

    double rGoalXVel = goalXVel_ * cos(angToGoal) + goalYVel_ * -sin(angToGoal);
    frc::SmartDashboard::PutNumber("RGXV", rGoalXVel);
    return rGoalXVel;

}

double SwerveDrive::getRGoalYVel()
{
    if(limelight_->hasTarget())
    {
        return goalYVel_;
    }

    if(goalY_ == 0 && goalX_ ==0)
    {
        return 0;
    }
    double angToGoal = (atan2(-goalY_, -goalX_) * 180 / M_PI);

    double rGoalYVel = goalXVel_ * sin(angToGoal) + goalYVel_ * cos(angToGoal);
    frc::SmartDashboard::PutNumber("RGYV", rGoalYVel);
    return rGoalYVel;
}