#pragma once

#include "vector"
#include <frc/Timer.h>
#include "TrajectoryPoint.h"

class SwervePath
{
    public:
        SwervePath(double maxLA, double maxLV, double maxAA, double maxAV);
        void setKLP(double klP);
        void setKLD(double klD);
        void setKAP(double kaP);
        void setKAD(double kaD);

        void setKLV(double klV);
        void setKLA(double klA);
        void setKAV(double kaV);
        void setKAA(double kaA);

        void generateTrajectory(bool spline);
        void generateLinearTrajectory();
        void generateSplineTrajectory();
        void addPoint(TrajectoryPoint point);

        std::pair<double, double> getPosition();
        std::pair<double, double> getVelocity();

    private:
        double MAX_LA, MAX_LV, MAX_AA, MAX_AV, klP_, klD_, kaP_, kaD_, klV_, klA_, kaV_, kaA_;

        std::vector<TrajectoryPoint> points_;
        double numPoints_;

        frc::Timer timer_;

};