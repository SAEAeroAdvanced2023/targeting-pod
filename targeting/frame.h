#ifndef FRAME_H
#define FRAME_H

#include <opencv2/core.hpp>

using namespace cv;

// TODO: Ill be fully honest, "Frame" is a bad name for this struct :(
struct Frame {
    time_t timestamp;
    Mat image;
};

#endif