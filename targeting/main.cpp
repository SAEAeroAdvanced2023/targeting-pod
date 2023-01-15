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

    // Misc variables
    string mode = "ready";
    Mat mask, mask1, mask2, hsv;
    vector<KeyPoint> keypoints;

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

        // Masking the frames
        cvtColor(camera.video, hsv ,COLOR_BGR2HSV);
        inRange(hsv, Scalar(0, 150, 150), Scalar(10, 255, 255), mask1);
        inRange(hsv, Scalar(170, 150, 150), Scalar(180, 255, 255), mask2);
        mask = mask1 | mask2; // bitwise or instead of addition!!!

        // Dectecting the blobd and drawing on the frame
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