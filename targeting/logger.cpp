#include <chrono>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <algorithm>

#include "logger.h"
#include "flightcontroller.h"
#include "imu.h"
#include "gimbal.h"

// TODO: Add some mutexes and pretty sure this becomes a thread safe singleton logger >:)

// Cute function to get datetime
std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d_%X", &tstruct);
    //std::string x(buf);
    //std::replace(x.begin(), x.end(), ':', '-');
    //std::cout << x << std::endl;
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
    logger.csvFile.open("./logs/" + s + ".csv");
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    double time = (double) std::chrono::duration_cast<std::chrono::nanoseconds> (now - logger.begin).count() / 1000000000;
    logger.logFile << "[" + std::to_string(time) + "] INIT: Logger started :)" << std::endl;
    logger.csvFile << "Timestamp,Latitude,Longitude,Altitude,Plane Roll,Plane Pitch,Plane Yaw,Gimbal Roll,Gimbal Pitch,Gimbal Yaw,Gimbal Delta Roll,Gimbal Delta Pitch,Gimbal Delta Yaw,X in frame,Y in frame,Estimated X,Estimated Y,Estimated Z" << std::endl;
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

// Logs captured info to csv
void Logger::logCSV(GPSPoint gpsPoint, CubeData cubeData, FlightController flightController, IMUData imuData, int pix_x, int pix_y) {
    //TODO: Delta logs just logging raw IMU
    logger.csvFile << gpsPoint.timestamp << "," << cubeData.latitude << "," << cubeData.longitude << "," << cubeData.altitude << "," << cubeData.roll << "," << cubeData.pitch << "," << cubeData.yaw << "," << toRad(imuData.roll) << "," << toRad(imuData.pitch) << "," << toRad(imuData.yaw) << "," << toRad(imuData.roll) - cubeData.roll + flightController.getInitData().roll << "," << toRad(imuData.pitch) - cubeData.pitch + flightController.getInitData().pitch << "," << toRad(imuData.yaw) - cubeData.yaw + flightController.getInitData().yaw << "," << pix_x << "," << pix_y << "," << gpsPoint.point(0,0) << "," << gpsPoint.point(1,0) << "," << gpsPoint.point(2,0) << std::endl;
}

// Closes the file, you should do this ;)
void Logger::closeLogger() {
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    double time = (double) std::chrono::duration_cast<std::chrono::nanoseconds> (now - logger.begin).count() / 1000000000;
    logger.logFile << "[" + std::to_string(time) + "] CLOSE: Closing Logger :)" << std::endl;
    logger.logFile.flush();
    logger.logFile.close();
    logger.csvFile.flush();
    logger.csvFile.close();
}
