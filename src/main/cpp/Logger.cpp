#include "Logger.h"

Logger::Logger(std::string fileName)
{
    fileName_ = fileName;
    open_ = false;
}

void Logger::openFile()
{
    outstream.open(fileName_);
    open_ = true;
    timer_.Reset();
    timer_.Start();
}

void Logger::closeFile()
{
    outstream.close();
    open_ = false;
    timer_.Stop();
}

void Logger::print(std::string print)
{
    if(!open_)
    {
        openFile();
    }

    double time = timer_.Get().value();
    outstream << time << ", " << print << std::endl;
}