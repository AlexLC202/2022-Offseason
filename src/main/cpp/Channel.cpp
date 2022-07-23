#include "Channel.h"

Channel::Channel()
{

}

void Channel::setColor(Color color)
{
    color_ = color;
}

Channel::Color Channel::getColor()
{
    return color_;
}

bool Channel::badIdea()
{
    frc::Color color = colorSensor_.GetColor();
    double proximity = colorSensor_.GetProximity();
    //frc::SmartDashboard::PutNumber("prox", proximity);

    //frc::SmartDashboard::PutNumber("r", color.red);
    //frc::SmartDashboard::PutNumber("g", color.green);
    //frc::SmartDashboard::PutNumber("b", color.blue);

    Color ballColor;
    if(color.red > 1.5 * color.blue)
    {
        ballColor = RED;
    }
    else if(color.blue > 1.5 * color.red)
    {
        ballColor = BLUE;
    }
    else
    {
        ballColor = UNKNOWN;
    }

    //frc::SmartDashboard::PutBoolean("RED", (ballColor == RED));
    //frc::SmartDashboard::PutBoolean("BLUE", (ballColor == BLUE));

    //0.394, 0.1856, red
    //0.5003, 0.1207, red
    //0.3907, 0.1861, red

    //0.2625, 0.2432, neither


    if(proximity < 310) //TODO get value
    {
        //frc::SmartDashboard::PutBoolean("BadIdea", false);
        return false;
    }

    bool badIdea = (ballColor != color_ && ballColor != UNKNOWN);
    //frc::SmartDashboard::PutBoolean("BadIdea", badIdea);

    //return false;
    return badIdea;
}