#include <iostream>
#include "flightcontroller.h"

#include "../mavlink/c_library_v2/common/mavlink.h"
#include "../mavlink/c_library_v2/common/mavlink_msg_attitude.h"
#include "../mavlink/c_library_v2/common/mavlink_msg_gps_raw_int.h"

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <thread>

using namespace std;

// TODO: Add "this" keyword in front of member variables to make code easier to understand :)
// TODO: MUTEXES

// Updates the sensor data
void FlightController::readData(){
    while(serial_port > 0) {
        read(serial_port, &byte, sizeof(byte));
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
            //std::cout << " ok, Received message with ID " << msg.msgid << ", sequence: " << (int) msg.seq << " from component " << (int) msg.compid << " of system " << (int) msg.sysid << std::endl;
            switch(msg.msgid) {
                // TODO: This is useless right? Remove if it is
                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: { // ID for GLOBAL_POSITION_INT
                    // Get all fields in payload (into global_position)
                    mavlink_msg_global_position_int_decode(&msg, &global_position);
                }
                    break;
                case MAVLINK_MSG_ID_GPS_RAW_INT: { // ID for raw gps data
                    // Get all fields in payload (into gps_raw_int)
                    mavlink_msg_gps_raw_int_decode(&msg, &gps_raw_int);
                    //std::cout <<"Lat: " << gps_raw_int.lat << ", lon: " << gps_raw_int.lon << ", alt: " << gps_raw_int.alt << std::endl;
                    data.latitude = gps_raw_int.lat;
                    data.longitude = gps_raw_int.lon;
                    data.altitude = gps_raw_int.alt;

                }
                    break;
                case MAVLINK_MSG_ID_ATTITUDE:{ // ID for raw attitude data
                    // Get all fields in payload (into attitude)
                    mavlink_msg_attitude_decode(&msg, &attitude);
                    //std::cout <<"roll: " << attitude.roll << ", pitch: " << attitude.pitch << ", yaw: " << attitude.yaw << std::endl;
                    data.roll = attitude.roll;
                    data.yaw = attitude.yaw;
                    data.pitch = attitude.pitch;
                }
                    break;
                default:
                    break;
            }
        }
    }
}

// Open serial connection
FlightController::FlightController(){

    // TODO: Dont hard code the serial port plz
    serial_port = open("/dev/ttyACM0", O_RDWR);
    if (serial_port < 0) {
        std::cout << "Error opening serial port to flight controller!!!" << std::endl;
    }

    std::thread updateDataThread(&FlightController::readData, this);

}

// TODO: Send a command to the controller (Do we ever do this from this program? Or is it only from the GS?)
void FlightController::sendData(){
    
}

CubeData FlightController::getData(){
    return this->data;
}