#ifndef IMU_H
#define IMU_H

struct IMUData {
    int x;
};

class IMU {
public:
    IMU();
    IMUData getSensorData();
private:
    IMUData data;
};

#endif