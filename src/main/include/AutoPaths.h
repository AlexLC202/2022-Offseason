#pragma once

#include <frc/Timer.h>

#include "SwerveDrive.h"
#include "Intake.h"
#include "Shooter.h"

class AutoPaths
{
    public:
        enum Path
        {
            TAXI_DUMB,
            TWO_DUMB,
            TWO_RIGHT,
            TWO_MIDDLE,
            TWO_LEFT,
            THREE,
            FIVE
        };
        void setPath(Path path);
        Path getPath();

        Shooter::State getShooterState();
        Intake::State getIntakeState();

        void startTimer();
        void stopTimer();

        void periodic(SwerveDrive* swerveDrive);
        double initYaw();
    private:
        Path path_;
        Shooter::State shooterState_;
        Intake::State intakeState_;

        frc::Timer timer_;
};