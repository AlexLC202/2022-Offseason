// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
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
    //frc::SendableChooser<std::string> m_chooser;
    //const std::string kAutoNameDefault = "Default";
    //const std::string kAutoNameCustom = "My Auto";
    //std::string m_autoSelected;

    frc::SendableChooser<Channel::Color> colorChooser;


    AHRS *navx_;
    Limelight* limelight_ = new Limelight();

    Controls* controls_ = new Controls();
    Channel* channel_ = new Channel();
    SwerveDrive* swerveDrive_ = new SwerveDrive(limelight_);
    Shooter* shooter_ = new Shooter(limelight_, channel_);
    Intake intake_;
    Climb climb_;
    

};
