//
// from: https://www.codeproject.com/Questions/1000793/Use-serial-port-object-in-Cplusplus-file
#include "../mavlink/c_library_v2/common/mavlink.h"
#include "../mavlink/c_library_v2/common/mavlink_msg_attitude.h"
#include "../mavlink/c_library_v2/common/mavlink_msg_gps_raw_int.h"
#include "../mavlink/c_library_v2/common/mavlink_msg_vfr_hud.h"
#include "../mavlink/c_library_v2/common/mavlink_msg_global_position_int.h"

#include <stdio.h>
#include <iostream>
#include <string.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

using namespace std;

// from: https://mavlink.io/en/mavgen_c/
mavlink_system_t mavlink_system = {
        1, // System ID (1-255)
        1  // Component ID (a MAV_COMPONENT value)
};

int main() {
    mavlink_status_t status;
    mavlink_message_t msg;
    mavlink_attitude_t attitude;
    mavlink_gps_raw_int_t gps_raw_int;
    mavlink_global_position_int_t global_position;
    mavlink_vfr_hud_t vfr_hud;
    int chan = MAVLINK_COMM_0;
    uint8_t byte;
    
        // Initialize serial port
    int serial_port = open("/dev/ttyACM0", O_RDWR);
    if (serial_port < 0) {
        std::cout << "Error opening serial p0rt" << std::endl;
        return 1;
    }
    
    while(serial_port > 0) {
        read(serial_port, &byte, sizeof(byte));
        if (mavlink_parse_char(chan, byte, &msg, &status)) {
            //std::cout << "hello" << std:: endl;
            //printf("Received message with ID %d, sequence: %d from component %d of system %d\n", msg.msgid, msg.seq, msg.compid, msg.sysid);
            std::cout << " ok, Received message with ID " << msg.msgid << ", sequence: " << (int) msg.seq << " from component " << (int) msg.compid << " of system " << (int) msg.sysid << std::endl;
            
            // ... DECODE THE MESSAGE PAYLOAD HERE ...
            switch(msg.msgid) {
                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: { // ID for GLOBAL_POSITION_INT
                    // Get all fields in payload (into global_position)
                    mavlink_msg_global_position_int_decode(&msg, &global_position);
                    }
                    break;
                case MAVLINK_MSG_ID_GPS_RAW_INT: { // ID for raw gps data 
                    // Get all fields in payload (into gps_raw_int)
                    mavlink_msg_gps_raw_int_decode(&msg, &gps_raw_int);
                    std::cout <<"Lat: " << gps_raw_int.lat << ", lon: " << gps_raw_int.lon << ", alt: " << gps_raw_int.alt << std::endl;
                    }    
                    break;
                case MAVLINK_MSG_ID_ATTITUDE:{ // ID for raw attitude data 
                    // Get all fields in payload (into attitude)
                    mavlink_msg_attitude_decode(&msg, &attitude);
                    std::cout <<"roll: " << attitude.roll << ", pitch: " << attitude.pitch << ", yaw: " << attitude.yaw << std::endl;
                    }
                    break;
                case 74:{
                    mavlink_msg_vfr_hud_decode(&msg, &vfr_hud);
                    std::cout << vfr_hud.alt << std::endl;
                    }
                    break;
                default:
                    break;
            }
        }
    }

    
}

