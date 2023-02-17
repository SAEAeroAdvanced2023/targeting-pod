#include <iostream>
#include <mutex>
#include "logger.h"
#include "flightcontroller.h"

using namespace std;

std::mutex flightControllerMutex;

// TODO: Add "this" keyword in front of member variables to make code easier to understand :)

// Updates the sensor data
void FlightController::readData(){
    
    Logger::logEvent("Starting Flight controller loop");
    
    bool inita = false;
    bool initb = false;
    
    while(this->serial_port > 0) {
        try {
            read(this->serial_port, &this->byte, sizeof(this->byte));       
        } catch(const std::exception& e){
            std::cout << e.what() << std::endl;
            Logger::logWarning("Flight controller couldn't read from port properly?!");
        }
        //std::cout << "port read!!!" << std::endl;
        //std::cout << "byte contents:" << (int) this->byte << std::endl;
        if (mavlink_parse_char(MAVLINK_COMM_0, this->byte, &this->msg, &this->status)) {
            //std::cout << " ok, Received message with ID " << this->msg.msgid << ", sequence: " << (int) this->msg.seq << " from component " << (int) this->msg.compid << " of system " << (int) this->msg.sysid << std::endl;
            switch(this->msg.msgid) {
                case MAVLINK_MSG_ID_GPS_RAW_INT: { // ID for raw gps data
                    // Get all fields in payload (into gps_raw_int)
                    mavlink_msg_gps_raw_int_decode(&this->msg, &this->gps_raw_int);
                    //std::cout <<"Lat: " << this->gps_raw_int.lat << ", lon: " << this->gps_raw_int.lon << ", alt: " << this->gps_raw_int.alt << std::endl;
                    if (!init) {
                        this->intData.latitude = this->gps_raw_int.lat;
                        this->intData.longitude = this->gps_raw_int.lon;
                        this->intData.altitude = this->gps_raw_int.alt;
                        inita = true;
                    }
                    if (flightControllerMutex.try_lock()){
                        this->data.latitude = this->gps_raw_int.lat;
                        this->data.longitude = this->gps_raw_int.lon;
                        this->data.altitude = this->gps_raw_int.alt;
                        flightControllerMutex.unlock();
                    }
                }
                    break;
                case MAVLINK_MSG_ID_ATTITUDE:{ // ID for raw attitude data
                    // Get all fields in payload (into attitude)
                    mavlink_msg_attitude_decode(&this->msg, &this->attitude);
                    //std::cout <<"roll: " << this->attitude.roll << ", pitch: " << this->attitude.pitch << ", yaw: " << this->attitude.yaw << std::endl;
                    if (!init) {
                        this->data.roll = this->attitude.roll;
                        this->data.yaw = this->attitude.yaw;
                        this->data.pitch = this->attitude.pitch;
                        initb = true;
                    }
                    if (flightControllerMutex.try_lock()) {
                        this->data.roll = this->attitude.roll;
                        this->data.yaw = this->attitude.yaw;
                        this->data.pitch = this->attitude.pitch;
                        flightControllerMutex.unlock();
                    }
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
    
    //Open and close python app.
    system("python3 CubeInit.py");
    //std::cout << "opening Serial port!!!" << std::endl;
    // Initialize serial port
    serial_port = open(flightControllerSerialPort.c_str(), O_RDWR);
    if (serial_port < 0) {
        std::cout << "Error opening serial p0rt" << std::endl;
        Logger::logCritical("Error opening serial port " + flightControllerSerialPort + " for flight controller");
        exit(1);
    } else {
        Logger::logEvent("Serial port " + flightControllerSerialPort + " opened");
    }
        std::thread updateDataThread(&FlightController::readData, this);
        updateDataThread.detach();

}

// TODO: Send a command to the controller (Do we ever do this from this program? Or is it only from the GS?)
void FlightController::sendData(){
    
}

CubeData FlightController::getData(){
    return this->data;
}

CubeData FlightController::getInitData(){
    return this->initData;
}

void FlightController::printData(){
    std::cout << "FlightController Data: " << std::endl;
    std::cout << "lat:" << this->data.latitude << "lon" << this->data.longitude << "alt" << this->data.altitude << std::endl;
    std::cout << "roll:" << this->data.roll << "yaw" << this->data.yaw << "pitch" << this->data.pitch << std::endl;
}
