#pragma once

#include <math.h>
#include "string"
#include "frc/Filesystem.h"

namespace GeneralConstants
{
    const double Kdt = 0.02;
    const int MAX_RPM = 6380;
    const int TICKS_PER_ROTATION = 2048;

    const double FREE_SPEED = 6380;
    const double FREE_CURRENT = 1.5;
    const double STALL_CURRENT = 257;
    const double MAX_VOLTAGE = 12;

    const double RESISTANCE = MAX_VOLTAGE/STALL_CURRENT;
    const double Kv = ((FREE_SPEED * 2 * M_PI) / 60 ) / (MAX_VOLTAGE - FREE_CURRENT * RESISTANCE);

    const double GOAL_HEIGHT = 2.641;

}

namespace LimelightConstants
{
    const double ANGLE_OFFSET = 90 - 49.5; //TODO get values more precise? 49.5 form 0 at top
    const double HEIGHT_OFFSET = 0.5334;
}

namespace InputConstants
{
    const int LJOY_PORT = 0;
    const int LJOY_X = 0;
    const int LJOY_Y = 1;

    const int RJOY_PORT = 1;
    const int RJOY_X = 0;
    const int RJOY_Y = 1;

    const int XBOX_PORT = 2;
    const int XBOX_LJOY_Y = 1;

    const int FIELD_ORIENT_BUTTON = 8;

    const int OUTAKE_BUTTON = 5; //TODO get value
    
    const int CLIMB_MODE_TOGGLE_BUTTON = 7;
    const int CLIMB_PNEUMATIC2_BUTTON = 3; //TODO get values
    const int CLIMB_PNEUMATIC1_BUTTON = 4; 
}

namespace SwerveConstants
{
    const double WIDTH = 29; 
    const double LENGTH = 29;
    const double TREAD_RADIUS = 0.0508; //TODO get value
    const double DRIVE_GEAR_RATIO = 1/6.12; //TODO get direction

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
    const double TL_CANCODER_OFFSET = -70.048 + 180;
    const double BR_CANCODER_OFFSET = 17.5 + 180;
    const double BL_CANCODER_OFFSET = 176.39 + 180;

}

namespace IntakeConstants
{
    const int MOTOR_ID = 40;
    const int SOLENOID_ID = 0;
}

namespace ClimbConstants
{
    const int MASTER_ID = 30;
    const int SLAVE_ID = 31;
    const int PNEUMATIC_1_ID = 3;
    const int PNEUMATIC_2_ID = 2;
    const int BRAKE_ID = 1;
}

namespace ShooterConstants
{
    
    const int HOOD_ID = 19;
    const int TURRET_ID = 20;
    const int KICKER_ID = 21;
    const int FLYWHEEL_SLAVE_ID = 22;
    const int FLYWHEEL_MASTER_ID = 23;

    const int MAX_HOOD_TICKS = -4300;
    const double MAX_HOOD_ANGLE = 70; //TODO get values
    const double MIN_HOOD_ANGLE = 53.5; //
    const double HOOD_ZERO_CURRENT = 2.8; //TODO test
    const double TICKS_PER_HOOD_DEGREE = -2150; //TODO get value

    const double FLYWHEEL_RADIUS = 0.0508; //TODO 2 inches, make more precise
    const double FLYWHEEL_GEAR_RATIO = (2.0/3.0); //TODO get value
    const double MAX_VELOCITY = (GeneralConstants::MAX_RPM / FLYWHEEL_GEAR_RATIO) * 2 * M_PI * FLYWHEEL_RADIUS / 60;

    const double TURRET_ZERO_CURRENT = 2.8; //TODO test value
    const double TICKS_PER_TURRET_DEGREE = 175; //TODO test value

    const std::string SHOTS_FILE_NAME = frc::filesystem::GetDeployDirectory() + "/shots.csv";

}