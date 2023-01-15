#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <iostream>
#include <string>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "json_struct.h"

using namespace std;
using namespace cv;

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
    JS_OBJ(minThreshold, maxThreshold, filterByArea, minArea, maxArea, filterByCircularity, minCircularity, filterByConvexity, minConvexity, filterByInertia, minInertiaRatio, filterByColor, blobColor)

};

string readFile(string file);
Ptr<SimpleBlobDetector> makeBlobParams(string input);

#endif