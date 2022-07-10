#include "TrajectoryPoint.h"

TrajectoryPoint::TrajectoryPoint(double x, double y, double yaw, double yawTime) : x_(x), y_(y), yaw_(yaw), yawTime_(yawTime)
{
    
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

double TrajectoryPoint::getYawTime()
{
    return yawTime_;
}