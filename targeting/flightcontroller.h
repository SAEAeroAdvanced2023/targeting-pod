#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include <iostream>

#include "../../mavlink/c_library_v2/common/mavlink.h"
#include "../../mavlink/c_library_v2/mavlink_helpers.h"
#include "../../mavlink/c_library_v2/mavlink_types.h"
#include "../../mavlink/c_library_v2/common/mavlink_msg_attitude.h"
#include "../../mavlink/c_library_v2/common/mavlink_msg_gps_raw_int.h"
#include "../../mavlink/c_library_v2/common/mavlink_msg_named_value_float.h"

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <mutex>
#include <thread>

#define FC_BUFFER_SIZE 100

using namespace std;

extern std::mutex flightControllerMutex;

struct CubeData{
    string mode = "READY";
    double latitude = 0;
    double longitude = 0;
    double altitude = 0;
    double roll = 0;
    double yaw = 0;
    double pitch = 0;
};

class FlightController{
public:
    FlightController(std::string port);
    void readData();
    void sendData(const char* name, float f);
    void printData();
    CubeData getData();
    CubeData getInitData();
private:
    CubeData data;
    CubeData initData;
    mavlink_status_t status;
    mavlink_message_t msg;
    mavlink_message_t sendMsg;
    mavlink_attitude_t attitude;
    mavlink_gps_raw_int_t gps_raw_int;
    mavlink_named_value_float_t namedFloat;
    uint8_t byte;
    uint8_t sendBuffer[FC_BUFFER_SIZE];
    int serial_port;
    int sendCount = 0;
    std::string flightControllerSerialPort;
};

#endif
