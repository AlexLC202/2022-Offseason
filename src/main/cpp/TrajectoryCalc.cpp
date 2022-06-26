#include "TrajectoryCalc.h"

TrajectoryCalc::TrajectoryCalc(double maxV, double maxA, double kP, double kD, double kV, double kA) : MAX_V(maxV), MAX_A(maxA), kP_(kP), kD_(kD), kV_(kV), kA_(kA)
{

}

void TrajectoryCalc::generateTrajectory(double pos, double setPos, double vel)
{
    timer_.Reset();
    timer_.Start();

    setPos_ = setPos;
    initPos_ = pos;
    initVel_ = vel;
    double error = setPos - pos;
    //prevAbsoluteError_ = error;
    if(error != 0)
    {
        direction_ = error / abs(error);
    }
    else
    {
        direction_ = -vel / abs(vel);
    }
    
    double initVelTime = vel / MAX_A;
    double initVelDistance = initVelTime * vel / 2;

    double distanceToAccel = abs(error) + abs(initVelDistance);
    distanceToAccel /= 2;

    cruiseSpeed_ = sqrt(2 * MAX_A * distanceToAccel);

    //cruiseSpeed_ = clamp(cruiseSpeed_, 0.0, MAX_V);
    if(cruiseSpeed_ > MAX_V)
    {
        cruiseSpeed_ = MAX_V;
    }

    accelDist_ = direction_ * ((cruiseSpeed_ * cruiseSpeed_) - (vel * vel)) / (2 * MAX_A);
    accelTime_ = abs((cruiseSpeed_ * direction_ - vel) / MAX_A);

    deccelDist_ = direction_ * (cruiseSpeed_ * cruiseSpeed_) / (2 * MAX_A);
    deccelTime_ = cruiseSpeed_ / MAX_A;

    if(cruiseSpeed_ != 0)
    {
        cruiseTime_ = abs((error - accelDist_ - deccelDist_) / cruiseSpeed_);
    }
    else
    {
        cruiseTime_ = 0;
    }
    cruiseDist_ = cruiseTime_ * cruiseSpeed_ * direction_;

}

tuple<double, double, double> TrajectoryCalc::getProfile()
{
    double time = timer_.Get().value();
    double tP, tV, tA;

    if (time <= accelTime_ && accelTime_ >= 0.02)
    {
        tA = MAX_A * direction_;
        tV = time * MAX_A * direction_ + initVel_;
        tP = ((tV + initVel_) / 2) * time + initPos_;
    }
    else if (time <= accelTime_ && accelTime_ < 0.02)
    {
        tA = 0;
        tV = initVel_;
        tP = initPos_;
    }
    else if (time > accelTime_ && time <= cruiseTime_ + accelTime_)
    {
        tA = 0;
        tV = cruiseSpeed_ * direction_;
        tP = accelDist_ + cruiseSpeed_ * direction_ * (time - accelTime_) + initPos_;
    }
    else if (time > cruiseTime_ + accelTime_ && time < deccelTime_ + accelTime_ + cruiseTime_)
    {
        tA = MAX_A * -direction_;
        tV = cruiseSpeed_ * direction_ - (time - accelTime_ - cruiseTime_) * MAX_A * direction_;
        tP = accelDist_ + cruiseDist_ + (time - accelTime_ - cruiseTime_) * (cruiseSpeed_ * direction_+ tV) / 2 + initPos_;
        //cout << cruiseSpeed_ << ", " << cruiseTime_ << ", " << tV << ", " << (time - accelTime_ - cruiseTime_) << endl;
    }
    else if (time > accelTime_ + deccelTime_ + cruiseTime_)
    {
        tA = 0;
        tV = 0;
        tP = setPos_;
    }
    else
    {
        tA = 0;
        tV = 0;
        tP = 0;
    }
    return tuple<double, double, double> (tA, tV, tP);

}

double TrajectoryCalc::calcPower(double pos, double vel)
{
    tuple<double, double, double> profile = getProfile();

    double error = get<2>(profile) - pos;

    //double absoluteError = (setPos_ - pos);
    //double deltaAbsoluteError = absoluteError - prevAbsoluteError_;
    //prevAbsoluteError_ = absoluteError;
    double velError = get<1>(profile) - vel;

    return (kP_ * error) + (kD_ * velError) + (get<1>(profile) * kV_) + (get<0>(profile) * kA_);
}