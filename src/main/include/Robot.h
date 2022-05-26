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
//#include "Channel.h"
#include "Shooter.h"
//#include "Climb.h"

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
    frc::SendableChooser<std::string> m_chooser;
    const std::string kAutoNameDefault = "Default";
    const std::string kAutoNameCustom = "My Auto";
    std::string m_autoSelected;

    AHRS *navx_;

    Controls *controls_ = new Controls();
    SwerveDrive swerveDrive_;
    Intake intake_;
    Shooter shooter_;
    //Channel channel_;
    //Climb climb_;

};
