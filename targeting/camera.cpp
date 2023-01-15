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
    this->video >> fr;

    Frame f;
    f.image = fr;
    f.timestamp = clock();

    return f;
}