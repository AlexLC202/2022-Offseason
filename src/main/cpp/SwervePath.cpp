#include "SwervePath.h"

SwervePath::SwervePath(double maxLA, double maxLV, double maxAA, double maxAV) : MAX_LA(maxLA), MAX_LV(maxLV), MAX_AA(maxAA), MAX_AV(maxAV)
{
    numPoints_ = 0;
}
        
void SwervePath::setKLP(double klP)
{
    klP_ = klP;
}
        
void SwervePath::setKLD(double klD)
{
    klD_ = klD;
}
        
void SwervePath::setKAP(double kaP)
{
    kaP_ = kaP;
}

void SwervePath::setKAD(double kaD)
{
    kaD_ = kaD;
}

void SwervePath::setKLV(double klV)
{
    klV_ = klV;
}

void SwervePath::setKLA(double klA)
{
    klA_ = klA;
}

void SwervePath::setKAV(double kaV)
{
    kaV_ = kaV;
}
void SwervePath::setKAA(double kaA)
{
    kaA_ = kaA;
}

void SwervePath::generateTrajectory(bool spline)
{
    if(spline)
    {
        generateSplineTrajectory();
    }
    else
    {
        generateLinearTrajectory();
    }
}

void SwervePath::generateLinearTrajectory()
{
    for(int i = 0; i < numPoints_ - 1; ++i)
    {
        TrajectoryPoint p1 = points_[i];
        TrajectoryPoint p2 = points_[i + 1];

        double dx = p2.getX() - p1.getX();
        double dy = p2.getY() - p1.getY();
        double dyaw = p2.getYaw() - p1.getYaw();
        double yawTime = p2.getYawTime();

        
    }
}

void SwervePath::generateSplineTrajectory()
{

}

void SwervePath::addPoint(TrajectoryPoint point)
{
    points_.push_back(point);
    ++numPoints_;
}

std::pair<double, double> SwervePath::getPosition()
{
    
}
    
std::pair<double, double> SwervePath::getVelocity()
{

}