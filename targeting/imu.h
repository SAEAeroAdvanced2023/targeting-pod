#ifndef IMU_H
#define IMU_H

#include <stdio.h>
#include <string.h>
#include <iostream>

#include "../../mavlink/c_library_v2/common/mavlink.h"
#include "../../mavlink/c_library_v2/common/mavlink_msg_gimbal_device_attitude_status.h"

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

struct IMUData {
    float_t w;
    float_t x;
    float_t y;
    float_t z;
};

class IMU {
public:
    IMU();
    IMUData getSensorData();
    void readSensorData();
private:
    int serial_port;
    IMUData data;
};

#endif
