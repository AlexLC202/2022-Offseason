#pragma once

#include <ctre/Phoenix.h>
#include <iostream>
#include <math.h>
#include "Controls.h"
#include "Constants.h"
#include "Limelight.h"

#include "SwerveModule.h"

#include <frc/smartdashboard/SmartDashboard.h>

class SwerveDrive
{
    public:
        SwerveDrive(Limelight* limelight);
        
        void periodic(double yaw, Controls* controls);
        void drive(double xSpeed, double ySpeed, double turn);

        void calcModules(double xSpeed, double ySpeed, double turn);

        void calcOdometry();
        //void calcOdometry2(); 
        void resetGoalOdometry(double turretAngle);
        void reset();
        bool foundGoal();
        void setFoundGoal(bool foundGoal);

        double getGoalX();
        double getGoalY();
        double getRGoalXVel();
        double getRGoalYVel();
        double getRobotGoalAng();
    private:
        SwerveModule* topRight_ = new SwerveModule(SwerveConstants::TR_TURN_ID, SwerveConstants::TR_DRIVE_ID, SwerveConstants::TR_CANCODER_ID, SwerveConstants::TR_CANCODER_OFFSET);
        SwerveModule* topLeft_ = new SwerveModule(SwerveConstants::TL_TURN_ID, SwerveConstants::TL_DRIVE_ID, SwerveConstants::TL_CANCODER_ID, SwerveConstants::TL_CANCODER_OFFSET);
        SwerveModule* bottomRight_ = new SwerveModule(SwerveConstants::BR_TURN_ID, SwerveConstants::BR_DRIVE_ID, SwerveConstants::BR_CANCODER_ID, SwerveConstants::BR_CANCODER_OFFSET);
        SwerveModule* bottomLeft_ = new SwerveModule(SwerveConstants::BL_TURN_ID, SwerveConstants::BL_DRIVE_ID, SwerveConstants::BL_CANCODER_ID, SwerveConstants::BL_CANCODER_OFFSET);

        double x_, y_, yaw_, goalX_, goalY_, yawOffset_, goalXVel_, goalYVel_;
        bool foundGoal_ = false;

        Limelight* limelight_;

        double trSpeed_, brSpeed_, tlSpeed_, blSpeed_, trAngle_, brAngle_, tlAngle_, blAngle_;

};