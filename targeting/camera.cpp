#include <iostream>
#include <ctime>
#include "camera.h"
#include "frame.h"

using namespace std;
using namespace cv;

// Constructor
Camera::Camera(){
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
    vid.open(0); // , cv::CAP_V4L2
    if (vid.isOpened()){
        cout << "Video opened fine" << endl;
        return vid;
    } else {
        cout << "ERROR: Video FAILED >:( >:(" << endl;
        exit(1);
    }
}
