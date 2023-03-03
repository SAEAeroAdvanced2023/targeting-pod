#ifndef GIMBAL_H
#define GIMBAL_H

#include <iostream>
#include <pigpio.h>
#include <opencv2/core.hpp>

using namespace std;

const int SERVO1 = 23;
const int SERVO2 = 24;
const int SERVO_MIN = 500;
const int SERVO_MID = 1500;
const int SERVO_MAX = 2500;
const int SERVO_INC = 3;

double toRad(double x);
double toDeg(double x);

class Gimbal{
public:
    Gimbal();
    void trackPoint(vector<cv::KeyPoint> keypoints, cv::Mat mask);
    void trackPointPolar(vector<cv::KeyPoint> keypoints, cv::Mat mask);
    void manualMove();
private:
    void initServos();
    int servo_limit(int x);
    int servo1_value;
    int servo2_value;
};

#endif
