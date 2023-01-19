#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include <iostream>

using namespace std;

//TODO: Define the data
struct CubeData{
    string mode;
    int q;
};

class FlightController{
public:
    FlightController();
    void readData();
    void sendData();
    CubeData getData();
private:
    CubeData data;
    int vehicle; // TODO: not an int but yunno
};

#endif