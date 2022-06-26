#pragma once
#include <math.h>

#include <frc/Timer.h>
#include "Constants.h"

class TrajectoryCalc
{
    public:
        TrajectoryCalc(double maxV, double maxA, double kP, double kD, double kV, double kA);

        void generateTrajectory(double pos, double setPos, double vel);
        tuple<double, double, double> getProfile();
        double calcPower(double pos, double vel);

    private:
        const double MAX_V, MAX_A;
        double kP_, kD_, kV_, kA_;

        double /*prevAbsoluteError_,*/ setPos_, initPos_, initVel_;

        int direction_;
        double cruiseSpeed_, cruiseDist_, accelDist_, deccelDist_, cruiseTime_, accelTime_, deccelTime_;

        frc::Timer timer_;
};