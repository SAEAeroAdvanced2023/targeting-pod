#include <iostream>
#include <vector>
#include "pointlist.h"

using namespace std;

// TODO: Do we even need to init anything here??
PointList::PointList(){

}

// Adds point to the list
void PointList::addPoint(GPSPoint p){
    this->pointList.push_back(p);
}

// Clears list of points
void PointList::clearList(){
    this->pointList.clear();
}

vector<GPSPoint> PointList::getPointList(){
    return this->pointList;
}