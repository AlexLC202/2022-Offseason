#include "Limelight.h"


Limelight::Limelight()
{
    table = nt::NetworkTableInstance::GetDefault().GetTable(tableName);
    //table->PutNumber("pipeline", PIPELINE);
}

double Limelight::getXOff()
{
    return table->GetNumber("tx", 10000.0);
}

double Limelight::getYOff()
{
    return table->GetNumber("ty", 10000.0);
}

bool Limelight::hasTarget()
{
    double targets = table->GetNumber("tv", -1);
    if(targets == -1)
    {
        return false;
    } 
    else 
    {
        return true;
    }
}


double Limelight::calcDistance()
{
    double y;
    if(!hasTarget())
    {
        return -1;
    }
    else
    {
        y = getYOff();
    }

    if(abs(getXOff()) > 10) //TODO get value, idk what the units or anything really is
    {
        return -1;
    }

    double angle = (y + LimelightConstants::ANGLE_OFFSET) * (M_PI / 180);

    return (GeneralConstants::GOAL_HEIGHT - LimelightConstants::HEIGHT_OFFSET) / tan(angle);
}

void Limelight::setLEDMode(std::string mode)
{
    if(mode == "OFF"){
        table->PutNumber("ledMode", 1);
    }
    if(mode == "BLINK"){
        table->PutNumber("ledMode", 2);
    }
    if(mode == "ON"){
        table->PutNumber("ledMode", 3);
    }
}