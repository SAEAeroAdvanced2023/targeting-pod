#include <chrono>
#include <iostream>
#include <fstream>

#include "logger.h"

// TODO: Add some mutexes and pretty sure this becomes a thread safe singleton logger >:)

// Cute function to get datetime
std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d_%X", &tstruct);
    return buf;
}

// Forward declaration
Logger Logger::logger;

// No need to do anything here
Logger::Logger(){}

// Initializes the logger
void Logger::initLogger(){
    std::string s = currentDateTime();
    logger.logFile.open("./logs/" + s + ".log");
    //logger.logFile.open("./logs/" + s + ".csv");
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    double time = (double) std::chrono::duration_cast<std::chrono::nanoseconds> (now - logger.begin).count() / 1000000000;
    logger.logFile << "[" + std::to_string(time) + "] INIT: Logger started :)" << std::endl;
}

// Use to log normal events
void Logger::logEvent(std::string x){
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    double time = (double) std::chrono::duration_cast<std::chrono::nanoseconds> (now - logger.begin).count() / 1000000000;
    logger.logFile << "[" + std::to_string(time) + "] EVENT: " + x << std::endl;
}

// Use for bad, but not program-crashing bad, things
void Logger::logWarning(std::string x){
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    double time = (double) std::chrono::duration_cast<std::chrono::nanoseconds> (now - logger.begin).count() / 1000000000;
    logger.logFile << "[" + std::to_string(time) + "] WARNING: " + x << std::endl;
}

// Use for something that makes plane explode
void Logger::logCritical(std::string x){
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    double time = (double) std::chrono::duration_cast<std::chrono::nanoseconds> (now - logger.begin).count() / 1000000000;
    logger.logFile << "[" + std::to_string(time) + "] CRITICAL: " + x << std::endl;
}

// Use for when you wanna write some debug stuff to log
void Logger::logDebug(std::string x){
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    double time = (double) std::chrono::duration_cast<std::chrono::nanoseconds> (now - logger.begin).count() / 1000000000;
    logger.logFile << "[" + std::to_string(time) + "] DEBUG: " + x << std::endl;
}

// Closes the file, you should do this ;)
void Logger::closeLogger() {
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    double time = (double) std::chrono::duration_cast<std::chrono::nanoseconds> (now - logger.begin).count() / 1000000000;
    logger.logFile << "[" + std::to_string(time) + "] CLOSE: Closing Logger :)" << std::endl;
    logger.logFile.flush();
    logger.logFile.close();
}
