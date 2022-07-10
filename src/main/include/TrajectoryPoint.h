#pragma once

class TrajectoryPoint
{
    public:
        TrajectoryPoint(double x, double y, double yaw, double yawTime);

        double getX();
        double getY();
        double getYaw();
        double getYawTime();
    private:
        double x_, y_, yaw_, yawTime_;

};