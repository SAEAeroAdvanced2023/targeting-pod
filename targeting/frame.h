#ifndef FRAME_H
#define FRAME_H

#include <opencv2/core.hpp>

using namespace cv;

struct Frame {
    time_t timestamp;
    Mat image;
};

#endif