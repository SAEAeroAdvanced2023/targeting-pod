#include <iostream>
#include "imu.h"

using namespace std;

IMU::IMU(){

    // Initialize serial port
    this->serial_port = open("/dev/ttyACM0", O_RDWR);
    if (this->serial_port < 0) {
        std::cout << "Error opening serial p0rt" << std::endl;
    }
}

// TODO: Updates the sensor data
void IMU::readSensorData(){
    
    mavlink_status_t status;
    mavlink_message_t msg;
    mavlink_gimbal_device_attitude_status_t gimbal_device_attitude_status;
    int chan = MAVLINK_COMM_0;
    uint8_t byte;
    
    while(serial_port > 0) {
        read(serial_port, &byte, sizeof(byte));
        if (mavlink_parse_char(chan, byte, &msg, &status)) {
            //std::cout << "Received message with ID " << msg.msgid << ", sequence: " << (int) msg.seq << " from component " << (int) msg.compid << " of system " << (int) msg.sysid << std::endl;
            
            // ... DECODE THE MESSAGE PAYLOAD HERE ...
            switch(msg.msgid) {
                case MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS: { // ID for IMU Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation).
                    // Get all fields in payload (into gps_raw_int)
                    mavlink_msg_gimbal_device_attitude_status_decode(&msg, &gimbal_device_attitude_status);
                    std::cout <<"Q[w, x, y, z]= [" << gimbal_device_attitude_status.q[0] 
                        << ", " << gimbal_device_attitude_status.q[1] 
                        << ", " << gimbal_device_attitude_status.q[2] 
                        << ", " << gimbal_device_attitude_status.q[3] << "]" << std::endl;
                        
                    this->data.w = gimbal_device_attitude_status.q[0];
                    this->data.x = gimbal_device_attitude_status.q[1];
                    this->data.y = gimbal_device_attitude_status.q[2];
                    this->data.z = gimbal_device_attitude_status.q[3];
                    }    
                    break;
                default:
                    break;
            }
        }
    }
}

// TODO: Updates Sensor data
IMUData IMU::getSensorData(){
    return this->data;
}

