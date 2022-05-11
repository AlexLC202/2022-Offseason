#pragma once

#include <math.h>

namespace GeneralConstants
{
    const double Kdt = 0.02;
    const int MAX_RPM = 6380;
    const int TICKS_PER_ROTATION = 2048;

    const double FREE_SPEED = 6380; //6380
    const double FREE_CURRENT = 1.5;
    const double STALL_CURRENT = 257;
    const double MAX_VOLTAGE = 12;

    const double RESISTANCE = MAX_VOLTAGE/STALL_CURRENT;
    const double Kv = ((FREE_SPEED * 2 * M_PI) / 60 ) / (MAX_VOLTAGE - FREE_CURRENT * RESISTANCE);

    const double GOAL_HEIGHT = 0.0; //TODO get values

    /*double rpmToTicksPerDS(double rpm)
    {
        return (rpm * TICKS_PER_ROTATION) / (600);
    }*/
}

namespace LimelightConstants
{
    const double ANGLE_OFFSET = 0.0; //TODO get values
    const double HEIGHT_OFFSET = 0.0;
}

namespace InputConstants
{
    const int LJOY_PORT = 0;
    const int LJOY_X = 0;
    const int LJOY_Y = 1;

    const int RJOY_PORT = 1;
    const int RJOY_X = 0;
    const int RJOY_Y = 1;

    const int OUTAKE_BUTTON = 0; //TODO get value
    
    const int CLIMB_TOGGLE_BUTTON = 0; //TODO get value
}

namespace SwerveConstants
{
    const double WIDTH = 29; 
    const double LENGTH = 29;
    const double TREAD_DIAMETER = 0.0; //TODO get value
    const double DRIVE_GEAR_RATIO = 1/6.12;

    const double trPosAngle = atan2((SwerveConstants::WIDTH/2), (SwerveConstants::LENGTH/2));
    const double tlPosAngle = -trPosAngle;
    const double brPosAngle = 180 - trPosAngle;
    const double blPosAngle = trPosAngle - 180;

    const int TR_DRIVE_ID = 13;
    const int TL_DRIVE_ID = 11;
    const int BR_DRIVE_ID = 18;
    const int BL_DRIVE_ID = 15;

    const int TR_TURN_ID = 14;
    const int TL_TURN_ID = 12;
    const int BR_TURN_ID = 17;
    const int BL_TURN_ID = 16;

    const int TR_CANCODER_ID = 62;
    const int TL_CANCODER_ID = 10;
    const int BR_CANCODER_ID = 8;
    const int BL_CANCODER_ID = 42;

    /*const double TR_CANCODER_OFFSET = 157.0;
    const double TL_CANCODER_OFFSET = -108.5;
    const double BR_CANCODER_OFFSET = -18.98;
    const double BL_CANCODER_OFFSET = 7.91;*/

    const double TR_CANCODER_OFFSET = 19.77;
    const double TL_CANCODER_OFFSET = -70.048;
    const double BR_CANCODER_OFFSET = 17.5;
    const double BL_CANCODER_OFFSET = 176.39;

}

namespace IntakeConstants
{
    const int MOTOR_ID = 50;
    const int SOLENOID_ID = 0;
}

namespace ChannelConstants
{
    const int MOTOR_ID = 40;
}

namespace ClimbConstants
{
    const int MASTER_ID = 0; //TODO get values
    const int SLAVE_ID = 0;
    const int PNEUMATIC_1_ID = 0;
    const int PNEUMATIC_2_ID = 0;
    const int BRAKE_ID = 0;
}

namespace ShooterConstants
{
    const int HOOD_ID = 0; //TODO get values
}