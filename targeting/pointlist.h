#ifndef POINTLIST_H
#define POINTLIST_H

#include <iostream>
#include <vector>

using namespace std;

// TODO: IDK what the point should be tbh
struct GPSPoint {
    double x;
    double y;
};

class PointList {
public:
    PointList();
    void addPoint(GPSPoint p);
    vector<GPSPoint> getPointList();
    void clearList();
private:
    vector<GPSPoint> pointList;
};

#endif