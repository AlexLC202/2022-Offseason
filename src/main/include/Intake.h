#pragma once

#include <ctre/Phoenix.h>
#include <iostream>
#include <math.h>
#include "Controls.h"
#include "Constants.h"

#include <frc/Solenoid.h>

class Intake
{
    public:
        enum State
        {
            INTAKING,
            OUTAKING,
            RETRACTED_IDLE,
            EXTENDED_IDLE,
        };

        State getState();
        void setState(State state);

        Intake();
        void periodic(); 
        void run(bool forward);
        void stop();
        void deploy();
        void retract();

    private:
        State state_;
        WPI_TalonFX* intakeMotor_ = new WPI_TalonFX(IntakeConstants::SOLENOID_ID);
        frc::Solenoid* intakePneumatic_ = new frc::Solenoid(frc::PneumaticsModuleType::CTREPCM, IntakeConstants::MOTOR_ID);

};