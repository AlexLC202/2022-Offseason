#pragma once

#include "Constants.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

class Channel
{
    public:
        enum Color
        {
            BLUE,
            RED,
            UNKNOWN
        };
        void setColor(Color color);
        Color getColor();

        Channel();
        //void periodic();

        int getBalls();
        bool badIdea();

    private:

        static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
        rev::ColorSensorV3 colorSensor_{i2cPort};
        //rev::ColorSensorV3 colorSensor_;

        Color color_;
};