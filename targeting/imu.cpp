// Interfacing with the Alexmos 8-bit gimbal controller with firmware v2.2b2
// Will NOT work with other firmwares, messages are different
// View systems docs for more info

#include <iostream>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <chrono>
#include <thread>

#include "imu.h"

using namespace std;

// TODO: MUTEXES
// TODO: Thread it just like the flight controller plz


// Updates the sensor data
void IMU::readSensorData(){
    
    const char command[] = {'\x3E', '\x44', '\x00', '\x44', '\x00'}; // Command to request data
    
    int header_checksum = 0;
    int body_checksum = 0;
    
    for (;;) { // Cool infinite loop notation >:)
        
        for (int i = 0; i < 5; i++) { // length of command 
            write(imu_port, &command[i], sizeof(command[i]));
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
            body_checksum += (int) message[i+4];
        }
        if (body_checksum % 256 != message[message[2] + 4]) {
            std::cout << "Body checksum invalid" << std::endl;
            continue;
        }
        
        // Crazy bitwise concatenation happening here (Little endian style)
        uint16_t uroll = (message[43] << 8) | (message[42]);
        this->data.roll = uroll / 10.0; // Keep the .0 or cast as float or we lose precision
        uint16_t upitch = (message[45] << 8) | (message[44]);
        this->data.pitch = upitch / 10.0;
        uint16_t uyaw = (message[47] << 8) | (message[46]);
        this->data.yaw = uyaw / 10.0;
        //std::cout << "Roll: " << (int16_t) uroll / 10.0 << " Pitch: " << (int16_t) upitch / 10.0 << " Yaw: " << (int16_t) uyaw / 10.0 << std::endl;
        
    }
    
}

IMU::IMU(){

    // Initialize serial port
    imu_port = open("/dev/ttyACM2",O_RDWR);
    if (imu_port < 0){
        cout << "Error opening IMU serial port!!!" << endl;
        exit(1);
    }
    
    std::thread updateIMUThread(&IMU::readSensorData, this);
    //updateIMUThread.join();
    updateIMUThread.detach();
    
}

// Returns Sensor data
IMUData IMU::getSensorData(){
    return this->data;
}

