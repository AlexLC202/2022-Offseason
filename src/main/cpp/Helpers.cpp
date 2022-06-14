#include "Helpers.h"

double Helpers::normalizeAngle(double& angle)
{
    angle += 360 * 10;
    angle = ((int)floor(angle) % 360) + (angle - floor(angle));
    angle -= 360 * floor(angle / 360 + 0.5);
}

//TODO include PID?