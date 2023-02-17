#ifndef CAMERA_H
#define CAMERA_H

#include <iostream>
#include <opencv2/videoio.hpp>
#include "frame.h"

using namespace std;
using namespace cv;

class Camera {
public:
    Camera(int port);
    Frame getFrame();
private:
    VideoCapture initVideo();
    VideoCapture video;
    int cameraPort
};

#endif
