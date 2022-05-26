#pragma once

#include <ctre/Phoenix.h>
#include <iostream>
#include <math.h>
#include "Controls.h"
#include "Constants.h"

#include "Limelight.h"
#include "Hood.h"
#include "Turret.h"

class Shooter
{
    public:
        enum State
        {
            IDLE,
            TRACKING, //TODO make tracking and aiming two?
            REVING,
            //SHOOTING,
            UNLOADING,
            MANUAL
        };
        State getState();
        void setState(State state);

        Shooter();
        void periodic();

        double linVelToSensVel(double velocity);
        double calcFlyPID(double velocity);
        
    private:
        Limelight limelight_;
        WPI_TalonFX flywheelMaster_, flywheelSlave_, kickerMotor_;
        //Turret turret_;
        Hood hood_;
        bool hoodZeroing_, flywheelReady_, shotReady_;

        State state_;

        double setPos_, prevError_, integralError_;
        double fKp_ = 0.0; //TODO tune values
        double fKi_ = 0.0;
        double fKd_ = 0;
};
