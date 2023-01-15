#ifndef GIMBAL_H
#define GIMBAL_H

#include <iostream>
#include <pigpio.h>

using namespace std;

// TODO: Decide whether or not to load from config file instead
const int H_SERVO = 23;
const int V_SERVO = 24;
const int SERVO_MIN = 500;
const int SERVO_MID = 1500;
const int SERVO_MAX = 2500;
const int SERVO_INC = 3;

class Gimbal{
public:
    Gimbal(int h, int v);
    void trackPoint(vector<KeyPoint> keypoints);
private:
    void initServos();
    int servo_limit(int x);
    int v_value;
    int h_value;
};

#endif