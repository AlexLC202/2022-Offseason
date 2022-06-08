#include "Limelight.h"


Limelight::Limelight()
{
    table = nt::NetworkTableInstance::GetDefault().GetTable(tableName);
    //table->PutNumber("pipeline", PIPELINE);
}

double Limelight::getXOff()
{
    frc::SmartDashboard::PutNumber("LX", table->GetNumber("tx", 10000.0));
    return table->GetNumber("tx", 10000.0);
}

double Limelight::getYOff()
{
    return table->GetNumber("ty", 10000.0);
}

bool Limelight::hasTarget()
{
    double targets = table->GetNumber("tv", -1);
    if(targets == -1 || targets == 0)
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
    if(!hasTarget() || abs(getXOff()) > 5) //TODO get value, also math if possible to remove
    {
        return -1;
    }
    
    y = getYOff();

    double angle = (y + LimelightConstants::ANGLE_OFFSET) * (M_PI / 180);

    return (GeneralConstants::GOAL_HEIGHT - LimelightConstants::HEIGHT_OFFSET) / tan(angle);
}

void Limelight::lightOn(bool light)
{
    if(light){
        table->PutNumber("ledMode", 3);
    }
    else{
        table->PutNumber("ledMode", 1);
    }
}