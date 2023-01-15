#include <iostream>
#include <pigpio.h>
#include "gimbal.h"

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
        gpioSetMode(V_SERVO, PI_OUTPUT);
        this.v_value = SERVO_MID;
        gpioServo(V_SERVO, this.v_value);
        gpioSetMode(H_SERVO, PI_OUTPUT);
        this.h_value = SERVO_MID;
        gpioServo(H_SERVO, this.h_value);
    }
}

void Gimbal::trackPoint(vector<KeyPoint> keypoints){
    if (keypoints.size() == 1){
            int hdif = abs(mask.cols/2 - keypoints[0].pt.x);
            if (keypoints[0].pt.x > mask.cols/2){
                h_value = servo_limit(h_value - (hdif * SERVO_INC));
                gpioServo(H_SERVO, h_value);
            } else {
                h_value = servo_limit(h_value + (hdif * SERVO_INC));
                gpioServo(H_SERVO, h_value);
            }
            int vdif = abs(mask.rows/2 - keypoints[0].pt.y);
            if (keypoints[0].pt.y > mask.rows/2){
                v_value = servo_limit(v_value - (vdif * SERVO_INC));
                gpioServo(V_SERVO, v_value);
            } else {
                v_value = servo_limit(v_value + (vdif * SERVO_INC));
                gpioServo(V_SERVO, v_value);
            }
        }
}