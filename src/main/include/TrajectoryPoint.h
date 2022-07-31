#pragma once

#include "Helpers.h"

class TrajectoryPoint
{
    public:
        TrajectoryPoint(double x, double y, double yaw, double yawDist);

        double getX();
        double getY();
        double getYaw();
        double getYawDist();
    private:
        double x_, y_, yaw_, yawDist_;

};