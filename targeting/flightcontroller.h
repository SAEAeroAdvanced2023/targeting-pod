#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include <stdio.h>
#include <string.h>
#include <iostream>

#include "../../mavlink/c_library_v2/common/mavlink.h"
#include "../../mavlink/c_library_v2/common/mavlink_msg_attitude.h"
#include "../../mavlink/c_library_v2/common/mavlink_msg_gps_raw_int.h"

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

using namespace std;

//TODO: Define the data
struct CubeData{
    string mode;
    int32_t lattitude = 0;
    int32_t longitude = 0;
    int32_t altitude = 0;
    int32_t roll = 0;
    int32_t yaw = 0;
    int32_t pitch = 0;
};

class FlightController{
public:
    FlightController();
    void readData();
    void sendData();
    CubeData getData();
private:
    int serial_port;
    CubeData data;
    int vehicle; // TODO: not an int but yunno
};

#endif
