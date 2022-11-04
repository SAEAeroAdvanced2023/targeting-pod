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

// TODO: Make a servo class with functions to read/write for clarity
int v_value;
int h_value;
// TODO: Decide whether or not to load from config file instead
const int H_SERVO = 23;
const int V_SERVO = 24;
const int SERVO_MIN = 500;
const int SERVO_MID = 1500;
const int SERVO_MAX = 2500;
const int SERVO_INC = 3;

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
};
JS_OBJ_EXT(BlobParams, minThreshold, maxThreshold, filterByArea, minArea, maxArea, filterByCircularity, minCircularity, filterByConvexity, minConvexity, filterByInertia, minInertiaRatio, filterByColor, blobColor)

// Prints crosshair at a specific coordinate
void crosshair(int x, int y, Mat frame, int r) {
    circle(frame, Point(x,y), r, Scalar(0,0,255), 2);
    circle(frame, Point(x,y), 1, Scalar(0,0,255), 1);
    line(frame, Point(x - r, y), Point(x - r/2 ,y), Scalar(0,0,255), 1);
    line(frame, Point(x + r, y), Point(x + r/2 ,y), Scalar(0,0,255), 1);
    line(frame, Point(x, y - r), Point(x ,y - r/2), Scalar(0,0,255), 1);
    line(frame, Point(x, y + r), Point(x ,y + r/2), Scalar(0,0,255), 1);
}

int servo_limit(int x){
    if (x > SERVO_MAX){
        x = SERVO_MAX;
    } else if (x < SERVO_MIN){
        x = SERVO_MIN;
    }
    return x;
}

void initServos(){
    if(gpioInitialise() < 0){ //Initialization Failed
        cout << "Could not init servos :( Exiting..." << endl;
        exit(1);
    } else {
        gpioSetMode(V_SERVO, PI_OUTPUT);
        v_value = SERVO_MID;
        gpioServo(V_SERVO, v_value);
        gpioSetMode(H_SERVO, PI_OUTPUT);
        h_value = SERVO_MID;
        gpioServo(H_SERVO, h_value);
    }
}

// Initializes Video
VideoCapture initVideo(){
    VideoCapture vid;
    vid.open(0); // If one webcam, almost always 0 as index
    if (vid.isOpened()){
        cout << "VideoCapture opened" << endl;
        return vid;
    } else {
        cout << "Could not open video :( Exiting..." << endl;
        exit(1);
    }
}

// Loads file into string
string readFile(string file){
    string paramText = "";
    string input;
    ifstream MyReadFile(file);
    while (getline (MyReadFile, input)) {
        paramText += input;
    }
    MyReadFile.close();
    return paramText;
}

// Creates BlobDetector from params in json format (Shout out json_struct.h)
Ptr<SimpleBlobDetector> makeBlobParams(string input) {
    BlobParams blobParams;
    JS::ParseContext parseContext(input);
    if (parseContext.parseTo(blobParams) != JS::Error::NoError) {
        std::string errorStr = parseContext.makeErrorString();
        fprintf(stderr, "Error parsing struct %s\n", errorStr.c_str());
        return nullptr;
    }
    SimpleBlobDetector::Params params;
    params.minThreshold = blobParams.minThreshold;
    params.maxThreshold = blobParams.maxThreshold;
    params.filterByArea = blobParams.filterByArea;
    params.minArea = blobParams.minArea;
    params.maxArea = blobParams.maxArea;
    params.filterByCircularity = blobParams.filterByConvexity;
    params.minCircularity = blobParams.minCircularity;
    params.filterByConvexity = blobParams.filterByConvexity;
    params.minConvexity = blobParams.minConvexity;
    params.filterByInertia = blobParams.filterByInertia;
    params.minInertiaRatio = blobParams.minInertiaRatio;
    params.filterByColor = blobParams.filterByColor;
    params.blobColor = blobParams.blobColor;
    Ptr <SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
    return detector;
}

int main(int argc, char** argv){

    // Init servos
    initServos();

    // Init the video
    VideoCapture vid = initVideo();

    // Loading params and creating blobDetector
    string paramText = readFile(BLOB_PARAM_FILE);
    Ptr<SimpleBlobDetector> detector = makeBlobParams(paramText);

    // Misc variables
    string mode = "ready";
    Mat frame, mask, mask1, mask2, hsv;
    vector<KeyPoint> keypoints;
    int vdif = 0;
    int hdif = 0;

    // Main loop
    while (true) {

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        vid >> frame;

        cvtColor(frame, hsv ,COLOR_BGR2HSV);
        inRange(hsv, Scalar(0, 150, 150), Scalar(10, 255, 255), mask1);
        inRange(hsv, Scalar(170, 150, 150), Scalar(180, 255, 255), mask2);
        mask = mask1 | mask2; // bitwise or instead of addition!!!

        detector->detect(mask, keypoints);
        drawKeypoints(mask,keypoints,mask);

        for (int i = 0; i < keypoints.size(); i++){
            crosshair(keypoints[i].pt.x, keypoints[i].pt.y, frame, 20);
            line(frame, Point(keypoints[i].pt.x, keypoints[i].pt.y), Point(mask.cols/2,mask.rows/2), Scalar(255,0,0), 1);
        }

        circle(frame, Point(mask.cols/2,mask.rows/2), 1, Scalar(0,0,255), 1);

        imshow("Preprocessed", frame);
        imshow("Masked", mask);

        // Tracking algorithm (BAD! PID needed)
        if (keypoints.size() < 0){
            hdif = abs(mask.cols/2 - keypoints[0].pt.x);
            if (keypoints[0].pt.x > mask.cols/2){
                h_value = servo_limit(h_value - (hdif * SERVO_INC));
                gpioServo(H_SERVO, h_value);
            } else {
                h_value = servo_limit(h_value + (hdif * SERVO_INC));
                gpioServo(H_SERVO, h_value);
            }
            vdif = abs(mask.rows/2 - keypoints[0].pt.y);
            if (keypoints[0].pt.y > mask.rows/2){
                v_value = servo_limit(v_value - (vdif * SERVO_INC));
                gpioServo(V_SERVO, v_value);
            } else {
                v_value = servo_limit(v_value + (vdif * SERVO_INC));
                gpioServo(V_SERVO, v_value);
            }
        }

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        double time = (double) std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() / 1000000000;
        cout << fixed << setprecision(10) << time << endl;

        if ((char)cv::waitKey(10) > 0) break;
    }

    return 0;

}
