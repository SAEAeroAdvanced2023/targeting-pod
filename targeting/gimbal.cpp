#include <iostream>
#include <pigpio.h>
#include <opencv2/core.hpp>
#include <cmath>
#include "gimbal.h"

// TODO: Function that combines setting servoX_value and writing in together

double toRad(double x){
    return x * M_PI/180;
}

Gimbal::Gimbal(){
    initServos();
}

int Gimbal::servo_limit(int x){
    if (x > SERVO_MAX){
        x = SERVO_MAX;
    } else if (x < SERVO_MIN){
        x = SERVO_MIN;
    }
    return x;
}

void Gimbal::initServos(){
    if(gpioInitialise() < 0){ //Initialization Failed
        cout << "Could not init servos :( Exiting..." << endl;
        exit(1);
    } else {
        gpioSetMode(SERVO1, PI_OUTPUT);
        this->servo1_value = SERVO_MID;
        gpioServo(SERVO1, this->v_value);
        gpioSetMode(SERVO2, PI_OUTPUT);
        this->servo2_value = SERVO_MID;
        gpioServo(SERVO2, this->h_value);
    }
}

// Automatically moves gimbal to track point (Pitch Roll configuration)
// TODO: Improve algorithm
void Gimbal::trackPoint(vector<cv::KeyPoint> keypoints, cv::Mat mask){
    if (keypoints.size() == 1){
            int hdif = abs(mask.cols/2 - keypoints[0].pt.x);
            if (keypoints[0].pt.x > mask.cols/2){
                servo1_value = servo_limit(servo1_value - (hdif * SERVO_INC));
                gpioServo(SERVO1, servo1_value);
            } else {
                servo1_value = servo_limit(servo1_value + (hdif * SERVO_INC));
                gpioServo(SERVO1, servo1_value);
            }
            int vdif = abs(mask.rows/2 - keypoints[0].pt.y);
            if (keypoints[0].pt.y > mask.rows/2){
                servo2_value = servo_limit(servo2_value - (vdif * SERVO_INC));
                gpioServo(SERVO2, servo2_value);
            } else {
                servo2_value = servo_limit(servo2_value + (vdif * SERVO_INC));
                gpioServo(SERVO2, servo2_value);
            }
        }
}

// Automatically moves gimbal to track point (Pitch Yaw configuration)
void Gimbal::trackPointPolar(vector<cv::KeyPoint> keypoints, cv::Mat mask){
    if (keypoints.size() == 1) {
        int theta = 0;
        int r = 0;
        // Keypoint coordinates start at the top left of the frame
        int x = keypoints[0].pt.x - (mask.cols/2);
        int y = (mask.cols/2) - keypoints[0].pt.y;

        r = std::sqrt((double) std::pow(x,2) + std::pow(y,2));
        // TODO: We can use data from the sensors and testing to make this a bit more accurate, no real reason to use SERVO_INC ;)
        theta = std::atan((double) (y/x)) * (180/M_PI) - 90; // Subtracting 90 degrees as our 0 degrees is along the positive y-axis

        servo1_value = servo_limit(servo1_value + ((theta/180) / (SERVO_MAX - SERVO_MID))); // Taking this as a percentage of the 180 degrees rotation
        gpioServo(SERVO1, servo1_value);
        servo2_value = servo_limit(servo2_value + r);
        gpioServo(SERVO2, servo2_value);
    }
}

// TODO: Implementation
void Gimbal::manualMove(){

}
