// Compile with: g++ *.cpp -lopencv_core -lopencv_videoio -lopencv_features2d -lopencv_highgui -lopencv_imgproc -lpthread -lpigpio


#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <pigpio.h>
#include <cstring>
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
#include "transmitter.h"

#include "json_struct.h"

// TODO: Make a general config for other things like camera index or servos enabled or not

// TODO: Stop using namespaces just in case
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

    // Init gimbal
    //Gimbal gimbal;

    // Init Camera
    Camera camera;

    // Init Flight Controller
    FlightController flightController;

    // Init IMU
    IMU imu;

    // Init PointList
    PointList pointList;

    // Init Transmitter
    Transmitter transmitter;

    // Load parameters and create detector (Shout out Jørgen Lind)
    string paramText = readFile(BLOB_PARAM_FILE);
    Ptr<SimpleBlobDetector> detector = makeBlobParams(paramText);

    // Misc variables
    cv::namedWindow("Display", CV_WINDOW_AUTOSIZE);
    string mode = "auto";
    Mat mask, mask1, mask2, hsv;
    vector<KeyPoint> keypoints;
    Frame frame;
    IMUData imuData;
    CubeData cubeData;
    
    // Tracking variables
    // TODO: Load from file, camera calibration tool should write matrix to file
    Eigen::MatrixXd ccm_inv(4,4);
    ccm_inv << 0.00204358, 0, -0.66121428, 0, 0, 0.00204224, -0.47667228, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    Eigen::MatrixXd ccm(3,3);
    ccm << 489.33767087, 0, 323.55705702, 0, 489.65953971, 233.40712684, 0, 0, 1;
    //TODO: Get actual values for these
    Eigen::MatrixXd v_dist(1,3);
    v_dist << 0, 0, -78;
    Eigen::MatrixXd g_dist(1,3);
    g_dist << 0, 0, 0;
    Eigen::MatrixXd c_dist(1,3);
    c_dist << 0, 0, 0;
    double f = 0.00304;
    Eigen::MatrixXd gnd(2,3);
    gnd << 1, 1, 0, 0, 0, 1;
    
    // Main Loop
    for (int i = 0; i < 100; i++){ // Use this for testing
    //while (true){
        cout << i << ": "; 
        // Just to time each frame (Optional)
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        // Get new frame
        frame = camera.getFrame();

        // Get new data from cube
        cubeData = flightController.getData();

        // Get new data from gimbal IMU
        imuData = imu.getSensorData();

        // TODO: Load color from Flight controller (Mavlink message im guessing? Might just go in flightController) 
        // Masking the frames
        cvtColor(frame.image, hsv ,COLOR_BGR2HSV);
        inRange(hsv, Scalar(0, 150, 150), Scalar(10, 255, 255), mask1);
        inRange(hsv, Scalar(170, 150, 150), Scalar(180, 255, 255), mask2);
        mask = mask1 | mask2; // Bitwise OR instead of addition!!!

        // Detecting the blob and drawing on the frame
        detector->detect(mask, keypoints);
        drawKeypoints(mask,keypoints,mask);

        for (int i = 0; i < keypoints.size(); i++){
            crosshair(keypoints[i].pt.x, keypoints[i].pt.y, frame.image, 20);
            line(frame.image, Point(keypoints[i].pt.x, keypoints[i].pt.y), Point(mask.cols/2,mask.rows/2), Scalar(255,0,0), 1);
        }

        crosshair(mask.cols/2, mask.rows/2, frame.image, 40);
        //circle(frame.image, Point(mask.cols/2,mask.rows/2), 1, Scalar(0,0,255), 1);

        // Display frames on screen
        imshow("Display", frame.image);
        imshow("Masked", mask);

        // Check flight controller data to see if mode changed (Unnecessary?)
        //mode = flightController.getData().mode;

        // Calculate the point and store it
        if ((mode == "auto" || mode == "manual") && keypoints.size() == 1){
            //pointList.addPoint(transform_dummy(frame.timestamp));
            pointList.addPoint(transform(v_dist, 0/*cubeData.roll*/, 0/*cubeData.yaw*/, 0/*cubeData.pitch*/, 0/*toRad(imuData.roll)*/, 0/*toRad(imuData.yaw)*/, 0/*toRad(imuData.pitch)*/ - (M_PI/2), ccm, ccm_inv, keypoints[0].pt.x, keypoints[0].pt.y, g_dist, c_dist, f, gnd, frame.timestamp));
        }

        // Move the gimbal
        if (mode == "ready" || mode == "auto"){
            //gimbal.trackPoint(keypoints, mask); // Pitch Roll config
            //gimbal.trackPointPolar(keypoints, mask); // Pitch Yaw config
        }

        // Print the amount of time the frame took to process (Optional)
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        double time = (double) std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() / 1000000000;
        cout << fixed << setprecision(10) << time << endl;
        cout << "Frame size (x,y): (" << mask.cols << "," << mask.rows << ")" << endl;

        //if ((char)cv::waitKey(10) > 0) break;
        cv::waitKey(30);

    }
    
    // Just for testing rn
    cout << pointList.calculateAverage() << endl;
    cout << imuData.pitch << endl;
    //gpioTerminate();

    return 0;

}
