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
#include <mutex>

#include "imu.h"
#include "logger.h"

using namespace std;

std::mutex imuMutex;

// Updates the sensor data
void IMU::readSensorData(){
    
    const char command[] = {'\x3E', '\x44', '\x00', '\x44', '\x00'}; // Command to request data
    
    int header_checksum = 0;
    int body_checksum = 0;
    
    Logger::logEvent("IMU sensor data loop started");
    
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
            Logger::logWarning("Header checksum from gimbal IMU invalid");
            continue;
        }
        body_checksum = 0;
        for (int i = 0; i < (int) message[2]; i++) {
            body_checksum += (int) message[i+4];
        }
        if (body_checksum % 256 != message[message[2] + 4]) {
            std::cout << "Body checksum invalid" << std::endl;
            Logger::logWarning("Body checksum from gimbal IMU invalid");
            continue;
        }
        
        // Crazy bitwise concatenation happening here (Little endian style)
        uint16_t uroll = (message[43] << 8) | (message[42]);
        uint16_t upitch = (message[45] << 8) | (message[44]);
        uint16_t uyaw = (message[47] << 8) | (message[46]);
        //std::cout << (int16_t) uroll % 3600 / 10.0 << " " << (int16_t) upitch / 10.0 << " " << (int16_t) uyaw / 10.0 << endl;
        double vroll = (int16_t) uroll % 3600 / 10.0;
        double vpitch = (int16_t) upitch % 3600 / 10.0;
        double vyaw = (int16_t) uyaw % 3600 / 10.0;
        vroll = vroll * (-1);
        vpitch = vpitch - 90;
        if (imuMutex.try_lock()) {
            this->data.roll = vroll;
            this->data.pitch = vpitch;
            this->data.yaw = vyaw;
            imuMutex.unlock();
        }
        //std::cout << "Roll: " << (int16_t) uroll / 10.0 << " Pitch: " << (int16_t) upitch / 10.0 << " Yaw: " << (int16_t) uyaw / 10.0 << std::endl;
        
    }
    
}

IMU::IMU(std::string port){

    this->IMUSerialPort = port;
    // Initialize serial port
    this->imu_port = open(IMUSerialPort.c_str() ,O_RDWR);
    if (imu_port < 0){
        cout << "Error opening IMU serial port!!!" << endl;
        Logger::logCritical("Could not open IMU serial port " + IMUSerialPort);
        exit(1);
    } else {
        Logger::logEvent("IMU serial port " + IMUSerialPort + " opened");
    }
    
    std::thread updateIMUThread(&IMU::readSensorData, this);
    updateIMUThread.detach();
    
}

// Returns Sensor data
IMUData IMU::getSensorData(){
    return this->data;
}

// Returns Sensor data
IMUData IMU::getInitSensorData(){
    return this->initData;
}

