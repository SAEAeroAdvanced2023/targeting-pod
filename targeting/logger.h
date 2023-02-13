// Singleton design pattern here!!! Don't try and create a Logger object or you will perish
//
// Usage:
// Logger::initLogger();
// Logger::logCritical("Plane Exploded");
// Logger::closeLogger();

#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <chrono>
#include <fstream>

class Logger {
public:
    static void initLogger();
    static void logEvent(std::string x);
    static void logWarning(std::string x);
    static void logCritical(std::string x);
    static void logDebug(std::string x);
    static void closeLogger();
private:
    Logger();
    static Logger logger;
    std::ofstream logFile;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
};

#endif