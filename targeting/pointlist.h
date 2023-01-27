#ifndef POINTLIST_H
#define POINTLIST_H

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

using namespace std;

// TODO: Add another field for certainty value for averaging
struct GPSPoint {
    Eigen::MatrixXd point;
    time_t timestamp;
};

class PointList {
public:
    PointList();
    void addPoint(GPSPoint p);
    vector<GPSPoint> getPointList();
    void clearList();
    Eigen::MatrixXd calculateAverage();
private:
    vector<GPSPoint> pointList;
};

#endif
