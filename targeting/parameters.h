#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <iostream>
#include <string>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "json_struct.h"

using namespace std;
using namespace cv;

struct BlobParams {
    int minThreshold;
    int maxThreshold;
    bool filterByArea;
    int minArea;
    int maxArea;
    bool filterByCircularity;
    float minCircularity;
    bool filterByConvexity;
    float minConvexity;
    bool filterByInertia;
    float minInertiaRatio;
    bool filterByColor;
    int blobColor;
    JS_OBJ(minThreshold, maxThreshold, filterByArea, minArea, maxArea, filterByCircularity, minCircularity, filterByConvexity, minConvexity, filterByInertia, minInertiaRatio, filterByColor, blobColor)

};

// Using this to get hsv color masking values
struct ColorParams {
    int minHue;
    int maxHue;
    int minSaturation;
    int maxSaturation;
    int minValue;
    int maxValue;
    JS_OBJ(minHue, maxHue, minSaturation, maxSaturation, minValue, maxValue)

};

// Vehicle config params
struct MathParams {
    double f; //camera Focal Length
    Eigen::MatrixXd ccm(3,3); //3x3 Camera Calibration Matrix
    Eigen::MatrixXd ccmInv(4,4); //4x4 inv Camera Calibration Matrix
    Eigen::MatrixXd vDist(1,3); //1x3 vehicle Distance from the origin
    Eigen::MatrixXd gDist(1,3); //1x3 gimbal distance from the vehicle
    Eigen::MatrixXd cDist(1,3); //1x3 camera distance from the gimbal
    Eigen::MatrixXd gnd(2,3); //2x3 Ground reference plane
};

// Vehicle config params
struct TempMathParams {
    double f; //camera Focal Length
    double ccm[9]; //3x3 Camera Calibration Matrix
    double ccmInv[16]; //4x4 inv Camera Calibration Matrix
    double vDist[3]; //1x3 vehicle Distance from the origin
    double gDist[3]; //1x3 gimbal distance from the vehicle
    double cDist[3]; //1x3 camera distance from the gimbal
    double gnd[6]; //2x3 Ground reference plane

    JS_OBJ(f, ccm, ccmInv, vDist, gDist, cDist, gnd)

};



// Vehicle config params
struct SystemParams {
    string imuPort;
    string fcPort;
    int cPort;

    JS_OBJ(imuPort, fcPort, cPort)

};

string readFile(string file);
Ptr<SimpleBlobDetector> makeBlobParams(string input);
ColorParams makeColorParams(string input);
MathParams makeMathParams(string input);

#endif