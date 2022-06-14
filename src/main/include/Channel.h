#pragma once

#include "Constants.h"

class Channel
{
    public:
        enum Color
        {
            BLUE,
            RED
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