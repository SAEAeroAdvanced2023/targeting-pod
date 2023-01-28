// Interfacing with the Alexmos 8-bit gimbal controller with firmware v2.2b2
// Will NOT work with other firmwares, messages are different
// View systems docs for more info

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <thread>

#include "imu.h"

using namespace std;

// TODO: MUTEXES
// TODO: Thread it just like the flight controller plz

IMU::IMU(){

    // Initialize serial port
    volatile int imu_port = open("/dev/ttyUSB0",0_RDWR);
    if (imu_port < 0){
        cout << "Error opening IMU serial port!!! << endl;"
        exit(1);
    }
    
}

// TODO: Updates the sensor data
void IMU::readSensorData(){
    
    int header_checksum = 0;
    int body_checksum = 0;
    
    for (;;) { // Cool infinite loop notation >:)
        
        for (char i : command) { // Only available in newer c++ versions? 
            write(imu_port, &i, sizeof(i));
        }
        
        // Wait for response from the gimbal board
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // DO NOT GO LOWER THAN 35 PLEASE OR PLANE EXPLODES 
        
        read(imu_port, &message, sizeof(message));
        
        // TODO: put this in a seperate function
        header_checksum = message[1] + message[2];
        if (header_checksum % 256 != message[3]) {
            std::cout << "Header checksum invalid" << std::endl;
            continue;
        }
        body_checksum = 0;
        for (int i = 0; i < (int) message[2]; i++) {
            body_checksum += (int) messaeg[i+4];
        }
        if (body_checksum % 256 != message[message[2] + 4]) {
            std::cout << "Body checksum invalid" << std::endl;
            continue;
        }
        
        // Crazy bitwise concatenation happening here (Little endian style)
        uint16_t uroll = (message[43] << 8) | (message[42]);
        roll = uroll / 10.0; // Keep the .0 or cast as float or we lose precision
        uint16_t upitch = (message[45] << 8) | (message[44]);
        pitch = upitch / 10.0;
        uint16_t uyaw = (message[47] << 8) | (message[46]);
        yaw = uyaw / 10.0;
        
    }
    
}

// Returns Sensor data
IMUData IMU::getSensorData(){
    return this->data;
}

