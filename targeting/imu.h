#ifndef IMU_H
#define IMU_H

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#define BUFFER_SIZE 60 // Size of the message; not sure how long it is but 60 seems to work fine. Increase this increases processing time but may also reduce invalid checksums

const std::string IMUSerialPort = "/dev/ttyACM0";

struct IMUData {
    float pitch = 0;
    float yaw = 0;
    float roll = 0;
};

class IMU {
public:
    IMU();
    IMUData getSensorData();
    void readSensorData();
private:
    volatile int imu_port;
    IMUData data;
    uint8_t message[BUFFER_SIZE];
};

#endif
