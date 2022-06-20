#pragma once

#include "Constants.h"
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
        rev::ColorSensorV3 colorSensor_;

        Color color_;
};