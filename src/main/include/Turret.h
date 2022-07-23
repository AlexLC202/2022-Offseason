#pragma once

#include <ctre/Phoenix.h>
#include <iostream>
#include <math.h>
#include "Controls.h"
#include "Constants.h"
#include "Limelight.h"
#include "Helpers.h"
#include "SwerveDrive.h"

class Turret
{
    public:
        enum State
        {
            IDLE,
            IMMOBILE,
            TRACKING,
            UNLOADING,
            MANUAL,
            CLIMB
        };
        State getState();
        void setState(State state);
        void setManualVolts(double manualVolts);
        bool isAimed();
        bool unloadReady();
        double getAngle();

        Turret(Limelight* limelight, SwerveDrive* swerveDrive);
        void periodic(double yaw, double offset/*, double robotGoalAng, bool foundGoal, double x, double y*/);
        void reset();

        void track();
        void calcUnloadAng();
        //void flip();

        double calcAngularFF();
        double calcLinearFF();
        double calcError();
        double calcPID();
        
    private:
        WPI_TalonFX turretMotor_;
        Limelight* limelight_;
        SwerveDrive* swerveDrive_;

        double maxV = 0;
        double maxA = 0;
        double kP = 0;
        double kD = 0;
        double kV = 0;
        double kA = 0;
        TrajectoryCalc trajectoryCalc_;
        bool initTrajectory_;

        State state_;
        double manualVolts_;
        bool aimed_, unloadReady_/*, foundGoal_*/;
        double prevYaw_, yaw_, offset_, /*robotGoalAng_, x_, y_, */unloadAngle_;

        double prevTime_, dT_;
        frc::Timer timer_;

        double prevError_, integralError_;
        double tkP_ = 0.2; //TODO tune, tuned with physics only, could be more aggressive
        double tkI_ = 0.0001;  //0.075, 0.0001, 0.000008
        double tkD_ = 0.000008;  //0.3, 0.0001, 0.00005
};