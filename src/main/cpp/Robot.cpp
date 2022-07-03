// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

Robot::Robot()
{
    timer.Start();

    AddPeriodic(
        [&]
        {
            //autoPaths_.periodic(swerveDrive_);

            //timer.Reset();
            cout << timer.GetFPGATimestamp().value() << endl;
            double yaw = navx_->GetYaw() - yawOffset_;
            Helpers::normalizeAngle(yaw);

            swerveDrive_->periodic(yaw, controls_);
            shooter_->periodic(-yaw);
            climb_.periodic(navx_->GetRoll());
        }, 5_ms, 2_ms);

    
    //AddPeriodic([&]{ cout << timer.Get().value() << endl; }, 5_ms, 2_ms);
}

void Robot::RobotInit()
{
    autoChooser_.SetDefaultOption("Taxi Dumb", AutoPaths::TAXI_DUMB);
    autoChooser_.AddOption("Two Dumb", AutoPaths::TWO_DUMB);
    autoChooser_.AddOption("Two Right", AutoPaths::TWO_RIGHT);
    autoChooser_.AddOption("Two Middle", AutoPaths::TWO_MIDDLE);
    autoChooser_.AddOption("Two Left", AutoPaths::TWO_LEFT);
    autoChooser_.AddOption("Three", AutoPaths::THREE);
    autoChooser_.AddOption("Five", AutoPaths::FIVE);
    frc::SmartDashboard::PutData("Auto Modes", &autoChooser_);

    colorChooser_.SetDefaultOption("Red", Channel::Color::RED);
    colorChooser_.SetDefaultOption("Blue", Channel::Color::BLUE);
    frc::SmartDashboard::PutData("Color", &colorChooser_);

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
    climb_.setPneumatics(false, false);
    limelight_->lightOn(true);
    navx_->ZeroYaw();
    channel_.setColor(colorChooser_.GetSelected());

    AutoPaths::Path path = autoChooser_.GetSelected();
    //m_autoSelected = frc::SmartDashboard::GetString("Auto Selector", kAutoNameDefault);
    //fmt::print("Auto selected: {}\n", m_autoSelected);
    autoPaths_.setPath(path);
    autoPaths_.startTimer();

    yawOffset_ = autoPaths_.initYaw();

}

void Robot::AutonomousPeriodic()
{
    limelight_->lightOn(true);
    //shooter_->setColor(colorChooser_.GetSelected());
    //channel_.setColor(colorChooser_.GetSelected());

    intake_.setState(autoPaths_.getIntakeState());
    shooter_->setState(autoPaths_.getShooterState());
    
    /*double yaw = navx_->GetYaw() - yawOffset_;
    Helpers::normalizeAngle(yaw);

    //swerveDrive_->periodic(yaw, controls_);
    swerveDrive_->setYaw(yaw);
    shooter_->periodic(-yaw);
    climb_.periodic(navx_->GetRoll());*/

    intake_.periodic();
}

void Robot::TeleopInit()
{
    controls_->setClimbMode(false);
    autoPaths_.stopTimer();
    channel_.setColor(colorChooser_.GetSelected());

    //odometryLogger_->openFile();
    flywheelLogger_->openFile();
    //hoodLogger_->openFile();
    //turretLogger_->openFile();

    //limelight_->lightOn(true);
    //climb_.setPneumatics(false, false);

    //frc::SmartDashboard::PutNumber("InV", 0);
    //frc::SmartDashboard::PutNumber("InA", 0);
    //frc::SmartDashboard::PutNumber("fKp", 0);
    frc::SmartDashboard::PutNumber("HINV", 0);
    frc::SmartDashboard::PutNumber("FINV", 0);

}

void Robot::TeleopPeriodic()
{
    //shooter_->setColor(colorChooser_.GetSelected());
    //channel_.setColor(colorChooser_.GetSelected());

    controls_->periodic();
    frc::SmartDashboard::PutBoolean("Climb Mode", controls_->getClimbMode());

    if(controls_->fieldOrient())
    {
        //navx_->Reset();
        navx_->ZeroYaw();
        yawOffset_ = 0;
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

        frc::SmartDashboard::PutBoolean("BAD IDEA", channel_.badIdea());

        if((channel_.badIdea() || shooter_->getState() == Shooter::UNLOADING) && !controls_->resetUnload())
        {
            //cout << "unloading state" << endl;
            shooter_->setState(Shooter::UNLOADING);
            intake_.setState(Intake::LOADING);
        }
        else if(controls_->shootPressed())
        {
            shooter_->setState(Shooter::REVING);
            intake_.setState(Intake::LOADING);
            //intake_.setState(Intake::INTAKING);
        }
        else
        {
            shooter_->setState(Shooter::TRACKING);
            intake_.setState(Intake::RETRACTED_IDLE);
        }
        //shooter_->setTurretManualVolts(controls_->getTurretManual());

        if(controls_->manuallyOverrideTurret())
        {
            shooter_->setState(Shooter::MANUAL);
        }

        if (controls_->intakePressed())
        {
            intake_.setState(Intake::INTAKING);
        }
        else if (controls_->outakePressed())
        {
           intake_.setState(Intake::OUTAKING);
        }
        else if(intake_.getState() != Intake::LOADING)
        {
            intake_.setState(Intake::RETRACTED_IDLE);
            //intake_.setState(Intake::EXTENDED_IDLE);
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
    
    /*stringstream odometry;
    odometry << swerveDrive_->getX() << ", " << swerveDrive_->getY() << ", " 
    << swerveDrive_->getSmoothX() << ", " << swerveDrive_->getSmoothY() << ", " 
    << swerveDrive_->getSWX() << ", " << swerveDrive_->getSWY();
    odometryLogger_->print(odometry.str());*/

    stringstream flywheel;
    flywheel << shooter_->getFlyPos() << ", " << shooter_->getFlyVel();
    flywheelLogger_->print(flywheel.str());

    /*stringstream hood;
    hood << shooter_->getHoodTicks();
    hoodLogger_->print(hood.str());

    stringstream turret;
    turret << shooter_->getTurretAngle();
    turretLogger_->print(turret.str());*/

    //frc::SmartDashboard::PutNumber("yaw", navx_->GetYaw());

    /*double yaw = navx_->GetYaw() - yawOffset_;
    Helpers::normalizeAngle(yaw);

    swerveDrive_->periodic(yaw, controls_);
    shooter_->periodic(-yaw);
    climb_.periodic(navx_->GetRoll());*/

    intake_.periodic();
}

void Robot::DisabledInit()
{
    shooter_->reset();
    //swerveDrive_->setFoundGoal(false);
    limelight_->lightOn(false);

    shooter_->setState(Shooter::IDLE);
    shooter_->periodic(-navx_->GetYaw());

    //navx_->Reset();
    navx_->ZeroYaw();
    yawOffset_ = 0;
    swerveDrive_->reset();

    //odometryLogger_->closeFile();
    flywheelLogger_->closeFile();
    //hoodLogger_->closeFile();
    //turretLogger_->closeFile();
}

void Robot::DisabledPeriodic() //TODO does this even do anything
{
    shooter_->reset();
    //swerveDrive_->setFoundGoal(false);
    limelight_->lightOn(false);

    //navx_->Reset();
    navx_->ZeroYaw();
    yawOffset_ = 0;
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
