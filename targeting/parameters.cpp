#include <iostream>
#include <fstream>
#include <iomanip>
#include "parameters.h"
#include "json_struct.h"

using namespace std;

// TODO: Load in camera calibration matrix from file and keep the calibration as a Python script that writes to file

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