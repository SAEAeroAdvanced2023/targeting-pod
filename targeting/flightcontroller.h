#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include <iostream>

using namespace std;

//TODO: Define the data
struct SensorData{
    int q;
};

class FlightController{
public:
    FlightController();
    void readData();
    void sendData();
private:
    SensorData data;
    int vehicle; // TODO: not an int but yunno
};

#endif