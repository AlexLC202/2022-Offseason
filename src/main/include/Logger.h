#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <frc/Timer.h>

#include "Constants.h"

class Logger
{
    public:
        Logger(std::string fileName);
        void openFile();
        void closeFile();
        void print(double print);
        void print(string print);
    private:
        std::ofstream outstream;
        std::string fileName_;

        bool open_;
        double startTime_;

        frc::Timer timer_; //TODO implement, also has red thing weird yeah
};