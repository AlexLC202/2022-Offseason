// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit()
{
    //m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
    //m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
    //frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

    colorChooser.SetDefaultOption("Red", Channel::Color::RED);
    colorChooser.SetDefaultOption("Blue", Channel::Color::BLUE);
    frc::SmartDashboard::PutData("Color", &colorChooser);

    controls_->setClimbMode(false);

    try
    {
        navx_ = new AHRS(frc::SPI::Port::kMXP);
    }
    catch (const std::exception &e)
    {
        std::cout << e.what() << std::endl;
    }
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit()
{
    //m_autoSelected = m_chooser.GetSelected();
    // m_autoSelected = SmartDashboard::GetString("Auto Selector",
    //     kAutoNameDefault);
    //fmt::print("Auto selected: {}\n", m_autoSelected);

    /*if (m_autoSelected == kAutoNameCustom)
    {
        // Custom Auto goes here
    }
    else
    {
        // Default Auto goes here
    }*/
}

void Robot::AutonomousPeriodic()
{
    /*if (m_autoSelected == kAutoNameCustom)
    {
        // Custom Auto goes here
    }
    else
    {
        // Default Auto goes here
    }*/
}

void Robot::TeleopInit()
{
    controls_->setClimbMode(false);
    //limelight_->lightOn(true);
    //climb_.setPneumatics(false, false);

}

void Robot::TeleopPeriodic()
{
    channel_->setColor(colorChooser.GetSelected());

    controls_->periodic();
    limelight_->lightOn(true);
    frc::SmartDashboard::PutBoolean("Climb Mode", controls_->getClimbMode());

    if(controls_->fieldOrient())
    {
        //navx_->Reset();
        navx_->ZeroYaw();
    }

    //TODO implement robot state machine?
    if(!controls_->getClimbMode())
    {
        //TODO remove, testing
        double fKp = frc::SmartDashboard::GetNumber("fKp", 0.0);
        double fKi = frc::SmartDashboard::GetNumber("fKi", 0.0);
        double fKd = frc::SmartDashboard::GetNumber("fKd", 0.0);

        double hKp = frc::SmartDashboard::GetNumber("hKp", 0.0);
        double hKi = frc::SmartDashboard::GetNumber("hKi", 0.0);
        double hKd = frc::SmartDashboard::GetNumber("hKd", 0.0);

        if(controls_->autoClimbPressed()) //TODO resusing buttons, remove later
        {
            shooter_->setPID(fKp, fKi, fKd);
            shooter_->setHoodPID(hKp, hKi, hKd);
        }

        climb_.setState(Climb::DOWN);
        climb_.setAutoState(Climb::UNINITIATED);
    
        if(controls_->increaseRange())
        {
            shooter_->increaseRange();
        }
        if(controls_->decreaseRange())
        {
            shooter_->decreaseRange();
        }

        if (controls_->intakePressed())
        {
            intake_.setState(Intake::INTAKING);
        }
        else if (controls_->outakePressed())
        {
           intake_.setState(Intake::OUTAKING);
        }
        else
        {
            intake_.setState(Intake::RETRACTED_IDLE);
        }


        if(controls_->shootPressed())
        {
            shooter_->setState(Shooter::REVING);
        }
        else
        {
            shooter_->setState(Shooter::TRACKING);
        }
    }
    else
    {
        intake_.setState(Intake::RETRACTED_IDLE);
        shooter_->setState(Shooter::MANUAL);
        shooter_->setTurretManualVolts(controls_->getTurretManual());

        if(climb_.getState() == Climb::IDLE || climb_.getState() == Climb::DOWN)
        {
            climb_.setState(Climb::MANUAL);
        }

        if(controls_->autoClimbPressed())
        {
            climb_.setState(Climb::AUTO);
            if(climb_.stageComplete())
            {
                climb_.readyNextStage();
            }
        }
        else if(controls_->autoClimbCancelled())
        {
            climb_.setState(Climb::MANUAL);
            climb_.setAutoState(Climb::UNINITIATED);
        }

        if(climb_.getState() == Climb::MANUAL)
        {
            if(controls_->getPneumatic1Toggle())
            {
                climb_.togglePneumatic1();
            }

            if(controls_->getPneumatic2Toggle())
            {
                climb_.togglePneumatic2();
            }

            climb_.extendArms(controls_->getClimbPower());
        }
        
    }
    

    //frc::SmartDashboard::PutNumber("yaw", navx_->GetYaw());
    swerveDrive_->periodic(navx_->GetYaw(), controls_);
    shooter_->periodic(-navx_->GetYaw(), swerveDrive_);
    intake_.periodic();
    climb_.periodic(navx_->GetPitch());
    //channel_.periodic();
}

void Robot::DisabledInit()
{
    shooter_->reset();
    //swerveDrive_->setFoundGoal(false);
    limelight_->lightOn(false);

    shooter_->setState(Shooter::IDLE);
    shooter_->periodic(-navx_->GetYaw(), swerveDrive_);

    //navx_->Reset();
    navx_->ZeroYaw();
    swerveDrive_->reset();
}

void Robot::DisabledPeriodic() //TODO does this even do anything
{
    shooter_->reset();
    //swerveDrive_->setFoundGoal(false);
    limelight_->lightOn(false);

    //navx_->Reset();
    navx_->ZeroYaw();
    swerveDrive_->reset();
}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
