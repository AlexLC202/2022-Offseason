// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <sstream>
#include <iostream>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "AHRS.h"
#include "Constants.h"
#include "Controls.h"
#include "SwerveDrive.h"
#include "Intake.h"
#include "Shooter.h"
#include "Limelight.h"
#include "Climb.h"
#include "Channel.h"
#include "Logger.h"
#include "AutoPaths.h"

class Robot : public frc::TimedRobot
{
public:
    void RobotInit() override;
    void RobotPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void TestInit() override;
    void TestPeriodic() override;

private:
    frc::SendableChooser<AutoPaths::Path> autoChooser_;
    frc::SendableChooser<Channel::Color> colorChooser_;


    AHRS *navx_;
    Limelight* limelight_ = new Limelight();

    Controls* controls_ = new Controls();
    SwerveDrive* swerveDrive_ = new SwerveDrive(limelight_);
    Shooter* shooter_ = new Shooter(limelight_, swerveDrive_);
    Intake intake_;
    Climb climb_;
    AutoPaths autoPaths_;

    //TODO test, also make not a pointer
    Logger* odometryLogger = new Logger(OutputConstants::odometryFile);
    Logger* hoodLogger = new Logger(OutputConstants::hoodFile);

    double yawOffset_;

};
