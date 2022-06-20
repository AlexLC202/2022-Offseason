#include "Channel.h"

Channel::Channel() : colorSensor_(ChannelConstants::COLOR_SENSOR_PORT)
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
    //double IR = colorSensor_.GetIR();
    double proximity = colorSensor_.GetProximity();

    if(proximity > 50) //TODO get value
    {
        frc::SmartDashboard::PutBoolean("BadIdea", false);
        return false;
    }

    /*if(color_ == Color::RED)
    {
        double rError = abs(ChannelConstants::RED_R - color.red);
        double gError = abs(ChannelConstants::RED_G - color.green);
        double bError = abs(ChannelConstants::RED_B - color.blue);

        return (abs(rError + gError + bError) > 40); //TODO get value
    }
    else
    {
        double rError = abs(ChannelConstants::BLUE_R - color.red);
        double gError = abs(ChannelConstants::BLUE_G - color.green);
        double bError = abs(ChannelConstants::BLUE_B - color.blue);

        return (abs(rError + gError + bError) < 40); //TODO get value
    }*/

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

    bool badIdea = (ballColor != color_);
    frc::SmartDashboard::PutBoolean("BadIdea", badIdea);

    return false; //TODO remove
    return (ballColor != color_);
}