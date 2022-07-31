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
        double yawDist = p2.getYawDist();
        double dyaw = p2.getYaw() - p1.getYaw();
        if(abs(dyaw) > 180)
        {
            dyaw = (dyaw > 0) ? dyaw -= 180 : dyaw += 180;
        }
        int yawDirection = (dyaw > 0) ? 1 : -1;
        dyaw = abs(dyaw);

        double yawAccelTime, yawCruiseTime, yawCruiseDist, yawCruiseVel, 
        linYawAccelTime, linYawCruiseTime, linYawCruiseDist, linYawCruiseVel, 
        linYawDeccelTime;
        if(dyaw != 0 && (dx != 0 || dy != 0))
        {
            yawCruiseVel = MAX_AA * 0.5 * sqrt(dyaw / (MAX_AA * 0.5));
            if(yawCruiseVel > MAX_AV * 0.5)
            {
                yawCruiseVel = MAX_AV * 0.5;
                yawAccelTime = MAX_AV / MAX_AA;
                double accelDeccelYaw = MAX_AV * yawAccelTime * 0.5;
                yawCruiseDist = dyaw - accelDeccelYaw;
                yawCruiseTime = yawCruiseDist / (MAX_AV * 0.5);
            }
            else
            {
                yawAccelTime = yawCruiseVel / (MAX_AA * 0.5);
                yawCruiseDist = 0;
                yawCruiseTime = 0;
            }

            double totalYawTime = yawAccelTime * 2 + yawCruiseTime;
            double linMaxAccelTime = MAX_LV / MAX_LA;
            double maxEndLinDist = sqrt(dx * dx + dy * dy) - yawDist;
            double maxEndLinVel = MAX_LA * sqrt(maxEndLinDist * 2 / MAX_LA); //Normal acceleration
            
            double linMaxDist;
            if(maxEndLinVel < MAX_LV * 0.5)
            {
                double maxDeccelTime = (MAX_LV * 0.5 - maxEndLinVel) / (MAX_LA * 0.5);
                if(maxDeccelTime + linMaxAccelTime >= totalYawTime)
                {
                    linMaxDist = 0.25 * MAX_LV * linMaxAccelTime + ((maxEndLinVel + MAX_LV * 0.5) / 2) * maxDeccelTime + MAX_LV * 0.5 * (totalYawTime - maxDeccelTime - linMaxAccelTime);
                }
                else if ((maxEndLinVel / (MAX_LA * 0.5)) > totalYawTime)
                {
                    linMaxDist = MAX_LA * 0.25 * totalYawTime * totalYawTime;
                }
                else
                {
                    double ghostEndTime = (maxEndLinVel / (MAX_LA * 0.5));
                    double halfGhostTime = (ghostEndTime + totalYawTime) / 2;
                    double maxVel = halfGhostTime * MAX_LA * 0.5;

                    linMaxDist = 0.5 * maxVel * halfGhostTime + (halfGhostTime - ghostEndTime) * ((maxVel + maxEndLinVel) / 2);
                }
            }
            else
            {
                if(totalYawTime > linMaxAccelTime)
                {
                    linMaxDist = 0.25 * MAX_LV * linMaxAccelTime + MAX_LV * 0.5 * (totalYawTime - linMaxAccelTime);
                }
                else
                {
                    linMaxDist = MAX_LA * 0.25 * totalYawTime * totalYawTime;
                }
            }

            if(linMaxDist < yawDist)
            {
                
            }
            else
            {

            }

        }
        else if(dx == 0 && dy == 0 && dyaw != 0)
        {
            yawCruiseVel = MAX_AA * sqrt(dyaw / MAX_AA);
            if(yawCruiseVel > MAX_AV)
            {
                yawCruiseVel = MAX_AV;
                yawAccelTime = MAX_AV / MAX_AA;
                double accelDeccelYaw = MAX_AV * yawAccelTime;
                yawCruiseDist = dyaw - accelDeccelYaw;
                yawCruiseTime = yawCruiseDist / MAX_AV;
            }
            else
            {
                yawAccelTime = yawCruiseVel / MAX_AA;
                yawCruiseDist = 0;
                yawCruiseTime = 0;
            }

            linYawAccelTime = 0;
            linYawCruiseTime = 0;
            linYawCruiseDist = 0;
            linYawCruiseVel = 0;
            linYawDeccelTime = 0;
        }
        else
        {
            yawAccelTime = 0;
            yawCruiseTime = 0;
            yawCruiseDist = 0;
            yawCruiseVel = 0;
            linYawAccelTime = 0;
            linYawCruiseTime = 0;
            linYawCruiseDist = 0;
            linYawCruiseVel = 0;
            linYawDeccelTime = 0;
        }
        //above
        
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

std::tuple<double, double, double> SwervePath::getPosition()
{
    
}
    
std::pair<double, double> SwervePath::getVelocity()
{

}