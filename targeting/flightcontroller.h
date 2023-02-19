#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include <iostream>

#include "../../mavlink/c_library_v2/common/mavlink.h"
#include "../../mavlink/c_library_v2/common/mavlink_msg_attitude.h"
#include "../../mavlink/c_library_v2/common/mavlink_msg_gps_raw_int.h"

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <mutex>
#include <thread>

using namespace std;



extern std::mutex flightControllerMutex;

//TODO: Define the data
struct CubeData{
    string mode;
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
    void sendData();
    void printData();
    CubeData getData();
    CubeData getInitData();
private:
    CubeData data;
    CubeData initData;
    mavlink_status_t status;
    mavlink_message_t msg;
    mavlink_attitude_t attitude;
    mavlink_gps_raw_int_t gps_raw_int;
    uint8_t byte;
    int serial_port;
    std::string flightControllerSerialPort;
};

#endif
