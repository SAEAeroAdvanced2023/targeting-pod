#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <pigpio.h>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "json_struct.h"

// TODO: Make a general config for other things like camera index or servos enabled or not
// TODO: Separate code into different files including header files

// TODO: Stop using namespaces
using namespace std;
using namespace cv;

const string BLOB_PARAM_FILE = "blobparams.json";
const string COLOR_PARAM_FILE = "colorparams.json";
const string MATH_PARAM_FILE = "mathparams.json";

// Prints crosshair at a specific coordinate
void crosshair(int x, int y, Mat frame, int r) {
    circle(frame, Point(x,y), r, Scalar(0,0,255), 2);
    circle(frame, Point(x,y), 1, Scalar(0,0,255), 1);
    line(frame, Point(x - r, y), Point(x - r/2 ,y), Scalar(0,0,255), 1);
    line(frame, Point(x + r, y), Point(x + r/2 ,y), Scalar(0,0,255), 1);
    line(frame, Point(x, y - r), Point(x ,y - r/2), Scalar(0,0,255), 1);
    line(frame, Point(x, y + r), Point(x ,y + r/2), Scalar(0,0,255), 1);
}

int main(int argc, char** argv){

    // Init gimbal
    Gimbal gimbal();

    // Init Camera
    Camera camera();

    // Init Flight Controller
    FlightController flightController();

    // Init IMU
    IMU imu();

    // Init PointList
    PointList pointList();

    // Init Transmitter
    Transmitter transmitter();

    // Load parameters and create detector (Shout out json_struct.h)
    string blobParams = readFile(BLOB_PARAM_FILE);
    Ptr<SimpleBlobDetector> detector = makeBlobParams(paramText);
    MathParams mathParams = makeMathParams(readFile(MATH_PARAM_FILE));
    ColorParams colorParams = makeColorParams(readFile(COLOR_PARAM_FILE));

    // Misc variables
    string mode = "ready";
    Mat mask, mask1, mask2, hsv;
    vector<KeyPoint> keypoints;

    Frame frame;
    IMUData imuData;
    CubeData cubeData;
    
    // Tracking variables
//    // TODO: Load from file, camera calibration tool should write matrix to file
//    Eigen::MatrixXd ccm_inv(4,4);
//    ccm_inv << 0.00204358, 0, -0.66121428, 0, 0, 0.00204224, -0.47667228, 0, 0, 0, 1, 0, 0, 0, 0, 1;
//    Eigen::MatrixXd ccm(3,3);
//    ccm << 489.33767087, 0, 323.55705702, 0, 489.65953971, 233.40712684, 0, 0, 1;
//    //TODO: Get actual values for these
//    Eigen::MatrixXd v_dist(1,3);
//    v_dist << 0, 0, -100;
//    Eigen::MatrixXd g_dist(1,3);
//    g_dist << 0, 0, 0;
//    Eigen::MatrixXd c_dist(1,3);
//    c_dist << 0, 0, 0;
//    double f = 0.304;
//    Eigen::MatrixXd gnd(2,3);
//    gnd << 1, 1, 0, 0, 0, 1;

    
    Logger::logEvent("Tracking loop started");

    // Main Loop
    while (true){

        // Just to time each frame (Optional)
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        // Get new frame
        camera.getFrame();

        // Get new data from data 
        flightController.readData();

        // Get new data from gimbal IMU
        imu.getSensorData();

        // Masking the frames using Json config file
        cvtColor(frame.image, hsv ,COLOR_BGR2HSV);
        if (colorParams.minHue>colorParams.maxHue){
            inRange(hsv, Scalar(0, colorParams.minScalar, colorParams.minValue), Scalar(colorParams.maxHue, colorParams.maxScalar, colorParams.maxValue), mask1);
            inRange(hsv, Scalar(colorParams.minHue, colorParams.minScalar, colorParams.minValue), Scalar(179, colorParams.maxScalar, colorParams.maxValue), mask2);
            mask = mask1 | mask2; // Bitwise OR instead of addition!!!
        } else{
            inRange(hsv, Scalar(colorParams.minHue, colorParams.minScalar, colorParams.minValue), Scalar(colorParams.maxHue, colorParams.maxScalar, colorParams.maxValue), mask);
        }
        // Detecting the blob and drawing on the frame
        detector->detect(mask, keypoints);
        drawKeypoints(mask,keypoints,mask);

        for (int i = 0; i < keypoints.size(); i++){
            crosshair(keypoints[i].pt.x, keypoints[i].pt.y, frame, 20);
            line(frame, Point(keypoints[i].pt.x, keypoints[i].pt.y), Point(mask.cols/2,mask.rows/2), Scalar(255,0,0), 1);
        }

        circle(frame, Point(mask.cols/2,mask.rows/2), 1, Scalar(0,0,255), 1);

        // Transmit video frame (With the points on plz)
        transmitter.transmitVideo();

        // Display frames on screen (optional)
        // imshow("Preprocessed", frame);
        // imshow("Masked", mask);

        // Check flight controller data to see if mode changed
        // IDK

        // Calculate the point and store it
        // pointList.addPoint(calculatePoint());

        // Check flight controller to see if Auto or Manual
        // IDK
        if ((mode == "auto" || mode == "manual") && keypoints.size() == 1){
            //pointList.addPoint(transform_dummy(frame.timestamp));
            GPSPoint m = transform(mathParams.v_dist, 0/*cubeData.roll*/, 0/*cubeData.yaw*/, 0/*cubeData.pitch*/-(M_PI/2), toRad(imuData.roll), toRad(imuData.yaw), toRad(imuData.pitch), mathParams.ccm, mathParams.ccm_inv, keypoints[0].pt.x, keypoints[0].pt.y, mathParams.g_dist, mathParams.c_dist, mathParams.f, mathParams.gnd, frame.timestamp);
            pointList.addPoint(m);
            Logger::logCSV(m, cubeData, imuData, keypoints[0].pt.x, keypoints[0].pt.y);
        }

        // Move the gimbal
        gimbal.trackPoint(keypoints);

        // Print the amount of thime the frame took to process (Optional)
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        double time = (double) std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() / 1000000000;
        cout << fixed << setprecision(10) << time << endl;

        if ((char)cv::waitKey(10) > 0) break;

    }

    gpioTerminate();

    return 0;

}