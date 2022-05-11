#pragma once

#include <iostream>
#include <math.h>
#include "Constants.h"

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"


class Limelight{
    public:
        Limelight();

        double getXOff();
        double getYOff();
        bool hasTarget();

        double calcDistance();

        void setLEDMode(std::string mode);
        std::shared_ptr<nt::NetworkTable> GetNetworkTable();

    private:
        void ReadPeriodicIn();

        std::shared_ptr<nt::NetworkTable> table;
        std::string tableName = "limelight";

        //const int PIPELINE = 0;
};