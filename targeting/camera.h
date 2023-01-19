#ifndef CAMERA_H
#define CAMERA_H

#include <iostream>
#include <opencv2/videoio.hpp>
#include "frame.h"

using namespace std;
using namespace cv;

class Camera {
public:
    Camera();
    Frame getFrame();
private:
    VideoCapture initVideo();
    VideoCapture video;
};

#endif