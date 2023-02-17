#include <iostream>
#include <fstream>
#include <iomanip>
#include "parameters.h"
#include "json_struct.h"
#include "logger.h"

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
        Logger::logCritical("Could not parse parameters for blob detection");
        exit(1);
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
    Logger::logEvent("Parameters parsed for blob detector");
    return detector;
}

// Creates ColorParams struct from json input (Shout out json_struct.h)
ColorParams makeColorParams(string input) {
    ColorParams colorParams;
    JS::ParseContext parseContext(input);
    if (parseContext.parseTo(colorParams) != JS::Error::NoError) {
        std::string errorStr = parseContext.makeErrorString();
        fprintf(stderr, "Error parsing struct %s\n", errorStr.c_str());
        Logger::logCritical("Could not parse color parameters");
        exit(1);
    }
    Logger::logEvent("Parameters parsed for color detection");
    return colorParams;
}

// Creates SystemParams struct from json input (Shout out json_struct.h)
SystemParams makeSystemParams(string input) {
    SystemParams systemParams;
    JS::ParseContext parseContext(input);
    if (parseContext.parseTo(systemParams) != JS::Error::NoError) {
        std::string errorStr = parseContext.makeErrorString();
        fprintf(stderr, "Error parsing struct %s\n", errorStr.c_str());
        Logger::logCritical("Could not parse color parameters");
        exit(1);
    }
    Logger::logEvent("Parameters parsed for system config");
    return systemParams;
}

// Creates MathParams struct from json input (Shout out json_struct.h)
MathParams makeMathParams(string input) {
    TempMathParams tempMathParams;
    MathParams mathParams;
    JS::ParseContext parseContext(input);
    if (parseContext.parseTo(tempMathParams) != JS::Error::NoError) {
        std::string errorStr = parseContext.makeErrorString();
        fprintf(stderr, "Error parsing struct %s\n", errorStr.c_str());
        Logger::logCritical("Could not parse math parameters");
        exit(1);
    }
    Logger::logEvent("Parameters parsed for math config");

    mathParams.ccm << TempMathParams.ccm; //3x3 Camera Calibration Matrix
    mathParams.ccmInv << TempMathParams.ccmInv; //4x4 inv Camera Calibration Matrix
    mathParams.vDist << TempMathParams.vDist; //1x3 vehicle Distance from the origin
    mathParams.gDist << TempMathParams.gDist; //1x3 gimbal distance from the vehicle
    mathParams.cDist << TempMathParams.cDist; //1x3 camera distance from the gimbal
    mathParams.gnd << TempMathParams.gnd; ; //2x3 Ground reference plane

    return mathParams;
}