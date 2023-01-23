#include <iostream>
#include "flightcontroller.h"

using namespace std;

// TODO: Connect to aircraft instance :)
FlightController::FlightController(){
        
    // Initialize serial port
    this->serial_port = open("/dev/ttyACM0", O_RDWR);
    if (this->serial_port < 0) {
        std::cout << "Error opening serial p0rt" << std::endl;
    }
}

// TODO: Updates the sensor data
void FlightController::readData(){
    
    mavlink_status_t status;
    mavlink_message_t msg;
    mavlink_attitude_t attitude;
    mavlink_gps_raw_int_t gps_raw_int;
    int chan = MAVLINK_COMM_0;
    uint8_t byte;
    
    while(serial_port > 0) {
        read(serial_port, &byte, sizeof(byte));
        if (mavlink_parse_char(chan, byte, &msg, &status)) {
            //std::cout << "Received message with ID " << msg.msgid << ", sequence: " << (int) msg.seq << " from component " << (int) msg.compid << " of system " << (int) msg.sysid << std::endl;
            
            // ... DECODE THE MESSAGE PAYLOAD HERE ...
            switch(msg.msgid) {
                case MAVLINK_MSG_ID_GPS_RAW_INT: { // ID for raw gps data 
                    // Get all fields in payload (into gps_raw_int)
                    mavlink_msg_gps_raw_int_decode(&msg, &gps_raw_int);
                    std::cout <<"Lat: " << gps_raw_int.lat << ", lon: " << gps_raw_int.lon << ", alt: " << gps_raw_int.alt << std::endl;
                    this->data.lattitude = gps_raw_int.lat;
                    this->data.longitude = gps_raw_int.lon;
                    this->data.altitude = gps_raw_int.alt;
                    }    
                    break;
                case MAVLINK_MSG_ID_ATTITUDE:{ // ID for raw attitude data 
                    // Get all fields in payload (into attitude)
                    mavlink_msg_attitude_decode(&msg, &attitude);
                    std::cout <<"roll: " << attitude.roll << ", pitch: " << attitude.pitch << ", yaw: " << attitude.yaw << std::endl;
                    this->data.roll = attitude.roll;
                    this->data.yaw = attitude.yaw;
                    this->data.pitch = attitude.pitch;
                    }
                    break;
                default:
                    break;
            }
        }
    }
}

// TODO: Send a command to the controller (Do we ever do this from this program? Or is it only from the GS?)
void FlightController::sendData(){
    
}

CubeData FlightController::getData(){
    return this->data;
}
