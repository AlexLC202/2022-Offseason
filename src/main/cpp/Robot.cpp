// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit()
{
    m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

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
    m_autoSelected = m_chooser.GetSelected();
    // m_autoSelected = SmartDashboard::GetString("Auto Selector",
    //     kAutoNameDefault);
    fmt::print("Auto selected: {}\n", m_autoSelected);

    if (m_autoSelected == kAutoNameCustom)
    {
        // Custom Auto goes here
    }
    else
    {
        // Default Auto goes here
    }
}

void Robot::AutonomousPeriodic()
{
    if (m_autoSelected == kAutoNameCustom)
    {
        // Custom Auto goes here
    }
    else
    {
        // Default Auto goes here
    }
}

void Robot::TeleopInit()
{
    navx_->ZeroYaw(); //COMP remove this if something in auto is added
    controls_->setClimbMode(false);
}

void Robot::TeleopPeriodic()
{
    controls_->periodic();

    if(controls_->fieldOrient())
    {
        navx_->Reset();
    }

    //TODO implement robot state machine?
    if(!controls_->getClimbMode())
    {
        //climb_.setExtendingState(Climb::extendingState::BRAKED);
        //climb_.setPneumaticState(Climb::pneumaticState::DOWN);

        if (controls_->intakePressed())
        {
            intake_.setState(Intake::State::INTAKING);
            //channel_.setState(Channel::State::INTAKING);
        }
        else if (controls_->outakePressed())
        {
           intake_.setState(Intake::State::OUTAKING);
            //channel_.setState(Channel::State::OUTAKING);
        }
        else
        {
            intake_.setState(Intake::State::RETRACTED_IDLE);
            //channel_.setState(Channel::State::IDLE);
        }


        if(controls_->shootPressed())
        {
            shooter_.setState(Shooter::REVING);
        }
        else
        {
            shooter_.setState(Shooter::TRACKING); //TODO change to tracking eventually
        }
    }
    /*else
    {
        intake_.setState(Intake::State::RETRACTED_IDLE);
        channel_.setState(Channel::State::IDLE);

        if(controls_->getClimbToggle())
        {
            if(climb_.getPneumaticState() == Climb::pneumaticState::DOWN || climb_.getPneumaticState() == Climb::pneumaticState::HALF_UP)
            {
                climb_.setPneumaticState(Climb::pneumaticState::UP);
            }
            else
            {
                climb_.setPneumaticState(Climb::pneumaticState::HALF_UP);
            }
        }
        
    }*/
    

    swerveDrive_.periodic(navx_->GetYaw(), controls_);
    intake_.periodic();
    shooter_.periodic();
    //channel_.periodic();
    //climb_.periodic(controls_);
}

void Robot::DisabledInit()
{
    //climb_.setPneumatics(true, false);
    //intake_.retract();
}

void Robot::DisabledPeriodic() 
{
    //shooter_.setState(Shooter::IDLE);
}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
