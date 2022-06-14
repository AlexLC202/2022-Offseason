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

    //calcOdometry();
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

void SwerveDrive::calcOdometry(double turretAngle)
{
    //resetGoalOdometry(turretAngle); //TODO change into this function if it works?

    if(!limelight_->hasTarget() && !foundGoal_)
    {
        return;
    }

    double frX = -topRight_->getDriveVelocity() * sin(topRight_->getAngle() * M_PI / 180);
    double frY = topRight_->getDriveVelocity() * cos(topRight_->getAngle() * M_PI / 180);
    double flX = -topLeft_->getDriveVelocity() * sin(topLeft_->getAngle() * M_PI / 180);
    double flY = topLeft_->getDriveVelocity() * cos(topLeft_->getAngle() * M_PI / 180);
    double brX = -bottomRight_->getDriveVelocity() * sin(bottomRight_->getAngle() * M_PI / 180);
    double brY = bottomRight_->getDriveVelocity() * cos(bottomRight_->getAngle() * M_PI / 180);
    double blX = -bottomLeft_->getDriveVelocity() * sin(bottomLeft_->getAngle() * M_PI / 180);
    double blY = bottomLeft_->getDriveVelocity() * cos(bottomLeft_->getAngle() * M_PI / 180);

    double avgX = (frX + flX + brX + blX) / 4;
    double avgY = (frY + flY + brY + blY) / 4;

    if(limelight_->hasTarget())
    {
        foundGoal_ = true;

        double distance = limelight_->calcDistance() + 0.6096; //Origin at goal center
        robotGoalAngle_ = (180 - (turretAngle + limelight_->getAdjustedX() + LimelightConstants::TURRET_ANGLE_OFFSET));
        double angleToGoal = yaw_ + robotGoalAngle_ + 90;
        x_ = -distance * cos(angleToGoal * M_PI / 180);
        y_ = -distance * sin(angleToGoal * M_PI / 180);
    }
    else
    {
        double angle = yaw_ * M_PI / 180;
    
        double rotatedX = avgX * cos(angle) + avgY * -sin(angle);
        double rotatedY = avgX * sin(angle) + avgY * cos(angle);

        x_ += rotatedX * GeneralConstants::Kdt;
        y_ += rotatedY * GeneralConstants::Kdt;

        if(x_ != 0 || y_ != 0)
        {
            robotGoalAngle_ = -yaw_ - 90 + atan2(-y_, -x_) * 180 / M_PI;
        }
        else
        {
            robotGoalAngle_ = 0;
        }
    }

    frc::SmartDashboard::PutNumber("x", x_);
    frc::SmartDashboard::PutNumber("y", y_);

    goalXVel_ = avgX * cos(robotGoalAngle_ * M_PI / 180) + avgY * sin(robotGoalAngle_ * M_PI / 180);
    goalYVel_ = avgX * -sin(robotGoalAngle_ * M_PI / 180) + avgY * cos(robotGoalAngle_ * M_PI / 180);

    /*double goalAngle = (-yaw_ - yawOffset_);
    goalAngle += 360 * 10;
    goalAngle = ((int)floor(goalAngle) % 360) + (goalAngle - floor(goalAngle));
    goalAngle -= 360 * floor(goalAngle / 360 + 0.5);
    
    goalAngle = goalAngle * M_PI / 180;

    goalXVel_ = avgX * cos(goalAngle) + avgY * -sin(goalAngle);
    goalYVel_ = avgX * sin(goalAngle) + avgY * cos(goalAngle);

    goalX_ += goalXVel_ * GeneralConstants::Kdt;
    goalY_ += goalYVel_ * GeneralConstants::Kdt;

    frc::SmartDashboard::PutNumber("gx", goalX_);
    frc::SmartDashboard::PutNumber("gy", goalY_);*/
}

/*void SwerveDrive::resetGoalOdometry(double turretAngle)
{
    if(!limelight_->hasTarget())
    {
        return;
    }

    foundGoal_ = true;
    goalY_ = - (limelight_->calcDistance() + 0.6096); //center of goal is origin
    goalX_ = 0;
    yawOffset_ = -yaw_ - (180 - (turretAngle + limelight_->getAdjustedX() + LimelightConstants::TURRET_ANGLE_OFFSET));
}*/

void SwerveDrive::reset()
{
    x_ = 0;
    y_ = 0;
    //goalX_ = 0;
    //goalY_ = 0;
    //yawOffset_ = 0;
    foundGoal_ = false;
}

double SwerveDrive::getRobotGoalAng()
{
    /*if(foundGoal_)
    {
        
        double robotGoalAng = -yaw_ - yawOffset_;

        robotGoalAng += 360 * 10;
        robotGoalAng = ((int)floor(robotGoalAng) % 360) + (robotGoalAng - floor(robotGoalAng));
        robotGoalAng -= 360 * floor(robotGoalAng / 360 + 0.5);

        return robotGoalAng;
    }
    else
    {
        return 0;
    }*/
    
    return robotGoalAngle_;
}

bool SwerveDrive::foundGoal()
{
    return foundGoal_;
}

void SwerveDrive::setFoundGoal(bool foundGoal)
{
    foundGoal_ = foundGoal;
}

double SwerveDrive::getX()
{
    return x_;
}

double SwerveDrive::getY()
{
    return y_;
}

/*double SwerveDrive::getGoalX()
{
    return goalX_;
}

double SwerveDrive::getGoalY()
{
    return goalY_;
}*/

double SwerveDrive::getGoalXVel() //TODO implement limelight distance if math works?
{
    /*if(limelight_->calcDistance() != -1)
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
    return rGoalXVel;*/

    return goalXVel_;

}

double SwerveDrive::getGoalYVel()
{
    /*if(limelight_->calcDistance() != -1)
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
    return rGoalYVel;*/

    return goalYVel_;
}