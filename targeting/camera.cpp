#include <iostream>
#include <ctime>
#include "camera.h"
#include "frame.h"
#include "logger.h"

using namespace std;
using namespace cv;

// Constructor
Camera::Camera(int port){
    this->cameraPort = port;
    this->video = this->initVideo();
    
}

// Returns a Frame struct
Frame Camera::getFrame(){
    Mat fr;
    //this->video >> fr;
    this->video.read(fr);

    Frame f;
    f.image = fr;
    f.timestamp = clock();

    return f;
}

VideoCapture Camera::initVideo(){
    VideoCapture vid;
    vid.open(this->cameraPort); // , cv::CAP_V4L2
    if (vid.isOpened()){
        cout << "Video opened fine" << endl;
        Logger::logEvent("Camera Initialized Successfully");
        return vid;
    } else {
        cout << "ERROR: Video FAILED >:( >:(" << endl;
        Logger::logCritical("Camera Initialization failed!!!");
        exit(1);
    }
}
