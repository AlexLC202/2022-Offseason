#include "Logger.h"

Logger::Logger(std::string fileName)
{
    fileName_ = fileName;
    open_ = false;
}

void Logger::openFile()
{
    outstream.open(fileName_, std::ofstream::out);
    //outstream.open(fileName_);
    open_ = true;
    startTime_ = timer_.GetFPGATimestamp().value();
    //timer_.Reset();
    //timer_.Start();
}

void Logger::closeFile()
{
    outstream.close();
    open_ = false;
    //timer_.Stop();
}


void Logger::print(double print)
{
    if(!open_)
    {
        openFile();
    }

    double time = timer_.GetFPGATimestamp().value() - startTime_;

    //cout << stod(print) << endl;

    outstream << time << ", " << print << endl;
}

void Logger::print(string print)
{
    if(!open_)
    {
        openFile();
    }

    double time = timer_.GetFPGATimestamp().value() - startTime_;

    //cout << stod(print) << endl;

    outstream << time << ", " << print << endl;
}