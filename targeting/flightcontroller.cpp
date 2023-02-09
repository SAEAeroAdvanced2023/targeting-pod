#include <iostream>
#include "flightcontroller.h"

using namespace std;

// TODO: Add "this" keyword in front of member variables to make code easier to understand :)
// TODO: MUTEXES

// Updates the sensor data
void FlightController::readData(){
    
    // TODO: Dont hard code the serial port plz
    
    std::cout << "thread started!!!" << std::endl;
    while(this->serial_port > 0) {
        try {
            //std::cout << "loop started!!!" << std::endl;
            read(this->serial_port, &this->byte, sizeof(this->byte));
            //std::cout << a << std::endl;            
        } catch(const std::overflow_error& e){
            std::cout << e.what() << std::endl;
        } catch(const std::runtime_error& e){
            std::cout << e.what() << std::endl;
        } catch(const std::exception& e){
            std::cout << e.what() << std::endl;
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
                    this->data.latitude = this->gps_raw_int.lat;
                    this->data.longitude = this->gps_raw_int.lon;
                    this->data.altitude = this->gps_raw_int.alt;

                }
                    break;
                case MAVLINK_MSG_ID_ATTITUDE:{ // ID for raw attitude data
                    // Get all fields in payload (into attitude)
                    mavlink_msg_attitude_decode(&this->msg, &this->attitude);
                    //std::cout <<"roll: " << this->attitude.roll << ", pitch: " << this->attitude.pitch << ", yaw: " << this->attitude.yaw << std::endl;
                    this->data.roll = this->attitude.roll;
                    this->data.yaw = this->attitude.yaw;
                    this->data.pitch = this->attitude.pitch;
                }
                    break;
                default:
                    break;
            }
        } else {
            //std::cout << "msg not received!!!" << std::endl;
        }

	//printData();

    }
}

// Open serial connection
FlightController::FlightController(){
    
    //Open and close python app.
    system("python3 CubeInit.py");
    //std::cout << "opening Serial port!!!" << std::endl;
    // Initialize serial port
    serial_port = open("/dev/ttyACM1", O_RDWR);
    if (serial_port < 0) {
        std::cout << "Error opening serial p0rt" << std::endl;
    } else {
        //std::cout << "Port opened!" << endl;
    }
        std::thread updateDataThread(&FlightController::readData, this);
        //updateDataThread.join();
        updateDataThread.detach();

}

// TODO: Send a command to the controller (Do we ever do this from this program? Or is it only from the GS?)
void FlightController::sendData(){
    
}

CubeData FlightController::getData(){
    return this->data;
}

void FlightController::printData(){
    std::cout << "FlightController Data: " << std::endl;
    std::cout << "lat:" << this->data.latitude << "lon" << this->data.longitude << "alt" << this->data.altitude << std::endl;
    std::cout << "roll:" << this->data.roll << "yaw" << this->data.yaw << "pitch" << this->data.pitch << std::endl;
}
