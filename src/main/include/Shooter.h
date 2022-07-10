#pragma once

#include <ctre/Phoenix.h>
#include <iostream>
#include <math.h>
#include <fstream>
#include <map>
#include "Controls.h"
#include "Constants.h"

#include "SwerveDrive.h"
#include "Limelight.h"
#include "Hood.h"
#include "Turret.h"
//#include "Channel.h"

using namespace std;

class Shooter
{
    public:
        enum State
        {
            IDLE,
            TRACKING,
            REVING,
            UNLOADING,
            MANUAL
        };
        State getState();
        void setState(State state);
        void setPID(double p, double i, double d);
        void setHoodPID(double p, double i, double d);
        void dewindIntegral();
        void increaseRange();
        void decreaseRange();
        //void setColor(Channel::Color color);
        void setTurretManualVolts(double manualVolts);
        double getHoodTicks();
        double getTurretAngle();
        double getFlyVel();

        Shooter(Limelight* limelight, SwerveDrive* swerveDrive);
        void periodic(double yaw);
        void createMap();
        void reset();

        double linVelToSensVel(double velocity);
        double calcFlyPID(double velocity);
        double calcFlyVolts(double velocity);

        tuple<double, double, double> calcShootingWhileMoving(double hoodAngle, double velocity, double goalXVel, double goalYVel);
        
    private:
        Limelight* limelight_;
        SwerveDrive* swerveDrive_;
        WPI_TalonFX flywheelMaster_, flywheelSlave_, kickerMotor_;

        double maxV = 20000;
        double maxA = 40000;
        double kP = 0;
        double kD = 0.0005;
        double kV = 1.0/1814.7555;
        double kA = 0;
        TrajectoryCalc flyTrajectoryCalc_;
        bool initTrajectory_;
        double setTrajectoryVel_;

        Turret turret_;
        Hood hood_;
        //Channel channel_;
        bool hoodZeroing_, flywheelReady_, flywheelEjectReady_, shotReady_, hasShot_;
        bool hasMap_  = false;
        double mapPoints_ = 0;
        double rangeAdjustment_ = 0;

        State state_;

        double prevTime_, dT_;
        frc::Timer timer_;

        double setPos_, prevError_, integralError_, prevVelocity_;
        double fKp_ = 0.0005; //TODO tune values
        double fKi_ = 0.00; 
        double fKd_ = 0.0000; 
        //0.0000001, 0.001, 0.00001
        //0.0001

        double yaw_;
        bool unloadShooting_, unloadShot_;

        map<double, tuple<double, double, double>> shotsMap_;
};
