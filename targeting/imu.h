#ifndef IMU_H
#define IMU_H

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <mutex>

#define BUFFER_SIZE 60 // Size of the message; not sure how long it is but 60 seems to work fine. Increase this increases processing time but may also reduce invalid checksums

extern std::mutex imuMutex;

struct IMUData {
    double pitch = 0;
    double yaw = 0;
    double roll = 0;
};

class IMU {
public:
    IMU(std::string port);
    IMUData getSensorData();
    void readSensorData();
private:
    volatile int imu_port;
    IMUData data;
    uint8_t message[BUFFER_SIZE];
    const std::string IMUSerialPort;
};

#endif
