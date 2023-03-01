// Compile with: g++ *.cpp -lopencv_core -lopencv_videoio -lopencv_features2d -lopencv_highgui -lopencv_imgproc -lpthread -lpigpio

#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <pigpio.h>
#include <cstring>
#include <mutex>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>

#include "camera.h"
#include "flightcontroller.h"
#include "frame.h"
#include "gimbal.h"
#include "imu.h"
#include "parameters.h"
#include "pointlist.h"
#include "transformer.h"
#include "logger.h"

#include "json_struct.h"

// TODO: Make a general config for other things like camera index or servos enabled or not
// TODO: Separate code into different files including header files

// TODO: Stop using namespaces
using namespace std;
using namespace cv;

const string BLOB_PARAM_FILE = "configs/blobparams.json";
const string COLOR_PARAM_FILE = "configs/colorparams.json";
const string MATH_PARAM_FILE = "configs/mathparams.json";
const string SYSTEM_PARAM_FILE = "configs/systemparams.json";

// Prints crosshair at a specific coordinate
void crosshair(int x, int y, Mat frame, int r) {
    circle(frame, Point(x,y), r, Scalar(0,0,255), 2);
    circle(frame, Point(x,y), 1, Scalar(0,0,255), 1);
    line(frame, Point(x - r, y), Point(x - r/2 ,y), Scalar(0,0,255), 1);
    line(frame, Point(x + r, y), Point(x + r/2 ,y), Scalar(0,0,255), 1);
    line(frame, Point(x, y - r), Point(x ,y - r/2), Scalar(0,0,255), 1);
    line(frame, Point(x, y + r), Point(x ,y + r/2), Scalar(0,0,255), 1);
}

/*int main(int argc, char *argv[]){
    Camera camera;
    Frame frame;
    int i = 0;
    while (true){
        frame = camera.getFrame();
        if(!frame.image.empty()){
            i++;
            cout << "W #" << i << endl;
            imshow("Preprocessed", frame.image);
        } else {
            i++;
            cout << "L #" << i << endl;
        }
        cv::waitKey(30);
    }
}*/

int main(int argc, char** argv){

    // If theres a cmd line param, append the first one to file name
    if (argc == 1) {
        Logger::initLogger("");
    } else {
        std::string sss(argv[1]);
        Logger::initLogger(sss);
    }

    // Load parameters and create detector (Shout out json_struct.h)
    Ptr<SimpleBlobDetector> detector = makeBlobParams(readFile(BLOB_PARAM_FILE));
    MathParams mathParams = makeMathParams(readFile(MATH_PARAM_FILE));
    ColorParams colorParams = makeColorParams(readFile(COLOR_PARAM_FILE));
    SystemParams systemParams = makeSystemParams(readFile(SYSTEM_PARAM_FILE));

    // Init gimbal
    //Gimbal gimbal();

    // Init Camera
    Camera camera(systemParams.cPort);

    // Init Flight Controller
    FlightController flightController(systemParams.fcPort);

    // Init IMU
    IMU imu(systemParams.imuPort);

    // Init PointList
    PointList pointList;

    // Misc variables
    cv::namedWindow("Display", CV_WINDOW_AUTOSIZE);
    string mode = "auto";
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
    /* Dummy values
    Eigen::MatrixXd v_dist(1,3);
    v_dist << 0, 0, -0.805;
    Eigen::MatrixXd g_dist(1,3);
    g_dist << 0, 0, 0;
    Eigen::MatrixXd c_dist(1,3);
    c_dist << 0, 0, 0;
    */
    
    Logger::logEvent("Tracking loop started");

    // Main Loop
    for (int i = 0; i < 100; i++){ // Use this for testing
    //while (true){

        // Just to time each frame (Optional)
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        // Get new frame
        frame = camera.getFrame();

        // Get new data from data 
        flightControllerMutex.lock();
        cubeData = flightController.getData();
        flightControllerMutex.unlock();

        // Get new data from gimbal IMU
        imuMutex.lock();
        imuData = imu.getSensorData();
        imuMutex.unlock();
        //Logger::logDebug();

        //cout << "Pitch: " << imuData.pitch << " Roll: " << imuData.roll << " Yaw: " << imuData.yaw << endl; 
        /*cout << "Pitch v: " << toDeg(cubeData.pitch - flightController.getInitData().pitch) 
            << " Roll v: " << toDeg(cubeData.roll - flightController.getInitData().roll) 
            << " Yaw v: " << toDeg(cubeData.yaw - flightController.getInitData().yaw) << endl;
        cout << "Pitch g: " << imuData.pitch - toDeg(cubeData.pitch - flightController.getInitData().pitch) 
            << " Roll g: " << imuData.roll - toDeg(cubeData.roll - flightController.getInitData().roll)  
            << " Yaw g: " << imuData.yaw - toDeg(cubeData.yaw - flightController.getInitData().yaw)  << endl; */

        // Masking the frames using Json config file
        cvtColor(frame.image, hsv ,COLOR_BGR2HSV);
        if (colorParams.minHue>colorParams.maxHue){
            inRange(hsv, Scalar(0, colorParams.minSaturation, colorParams.minValue), Scalar(colorParams.maxHue, colorParams.maxSaturation, colorParams.maxValue), mask1);
            inRange(hsv, Scalar(colorParams.minHue, colorParams.minSaturation, colorParams.minValue), Scalar(179, colorParams.maxSaturation, colorParams.maxValue), mask2);
            mask = mask1 | mask2; // Bitwise OR instead of addition!!!
        } else{
            inRange(hsv, Scalar(colorParams.minHue, colorParams.minSaturation, colorParams.minValue), Scalar(colorParams.maxHue, colorParams.maxSaturation, colorParams.maxValue), mask);
        }
        
        // Detecting the blob and drawing on the frame
        detector->detect(mask, keypoints);
        drawKeypoints(mask,keypoints,mask);

        for (int i = 0; i < keypoints.size(); i++){
            crosshair(keypoints[i].pt.x, keypoints[i].pt.y, frame.image, 20);
            line(frame.image, Point(keypoints[i].pt.x, keypoints[i].pt.y), Point(mask.cols/2,mask.rows/2), Scalar(255,0,0), 1);
        }

        crosshair(mask.cols/2, mask.rows/2, frame.image, 40);
        //circle(frame, Point(mask.cols/2,mask.rows/2), 1, Scalar(0,0,255), 1);

        // Display frames on screen
        imshow("Display", frame.image);
        imshow("Masked", mask);

        // Check flight controller data to see if mode changed (Unnecessary?)
        //mode = flightController.getData().mode;

        // Calculate the point and store it
        if ((mode == "auto" || mode == "manual") && keypoints.size() == 1){
            //pointList.addPoint(transform_dummy(frame.timestamp));

            GPSPoint m = transform(mathParams.vDist, cubeData.roll - flightController.getInitData().roll, cubeData.yaw - flightController.getInitData().yaw, cubeData.pitch - flightController.getInitData().pitch, toRad(imuData.roll) - cubeData.roll + flightController.getInitData().roll, toRad(imuData.yaw) - cubeData.yaw + flightController.getInitData().yaw, toRad(imuData.pitch) - cubeData.pitch + flightController.getInitData().pitch, mathParams.ccm, mathParams.ccmInv, keypoints[0].pt.x, keypoints[0].pt.y, mathParams.gDist, mathParams.cDist, mathParams.f, mathParams.gnd, frame.timestamp);
            pointList.addPoint(m);
            Logger::logCSV(m, cubeData, flightController, imuData, keypoints[0].pt.x, keypoints[0].pt.y);
        }

        // Move the gimbal
        if (mode == "ready" || mode == "auto"){
            //gimbal.trackPoint(keypoints, mask); // Pitch Roll config
            //gimbal.trackPointPolar(keypoints, mask); // Pitch Yaw config
        }

        // Print the amount of time the frame took to process (Optional)
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        double time = (double) std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() / 1000000000;
        cout << i << ": " << fixed << setprecision(10) << time << endl;
        Logger::logDebug("Frame " + std::to_string(i) + " process time: " + std::to_string(time) + "s");

        //if ((char)cv::waitKey(10) > 0) break;
        cv::waitKey(30);

    }

        // Just for testing rn
    Eigen::MatrixXd m = pointList.calculateAverage();
    cout << m << endl;
    Logger::logEvent("Calculated point: " + vec2string(m));
    //gpioTerminate();

    Logger::closeLogger();

    return 0;

}
