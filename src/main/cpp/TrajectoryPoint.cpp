#include "TrajectoryPoint.h"

TrajectoryPoint::TrajectoryPoint(double x, double y, double yaw, double yawDist) : x_(x), y_(y), yaw_(yaw), yawDist_(yawDist)
{
    Helpers::normalizeAngle(yaw);
}

double TrajectoryPoint::getX()
{
    return x_;
}

double TrajectoryPoint::getY()
{
    return y_;
}

double TrajectoryPoint::getYaw()
{
    return yaw_;
}

double TrajectoryPoint::getYawDist()
{
    return yawDist_;
}