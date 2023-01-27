#include <iostream>
#include <vector>
#include <Eigen/Dense>
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

Eigen::MatrixXd PointList::calculateAverage(){
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    int size  = this->pointList.size();
    for (int i = 0; i < size; i++){
        x += this->pointList[i].point(0,0);
        //std::cout << "x:" << x << std::endl;
        y += this->pointList[i].point(1,0);
        //std::cout << "y:" << y << std::endl;
        z += this->pointList[i].point(2,0);
        //std::cout << "z:" << z << std::endl;
    }
    x /= size;
    y /= size;
    z /= size;
    Eigen::MatrixXd a(1,3);
    a << x, y, z;
    return a;
}
