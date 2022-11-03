#include <iostream>
#include <iomanip>
#include <chrono>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

void crosshair(int x, int y, Mat frame, int r) {
    circle(frame, Point(x,y), r, Scalar(0,0,255), 2);
    circle(frame, Point(x,y), 1, Scalar(0,0,255), 1);
    line(frame, Point(x - r, y), Point(x - r/2 ,y), Scalar(0,0,255), 1);
    line(frame, Point(x + r, y), Point(x + r/2 ,y), Scalar(0,0,255), 1);
    line(frame, Point(x, y - r), Point(x ,y - r/2), Scalar(0,0,255), 1);
    line(frame, Point(x, y + r), Point(x ,y + r/2), Scalar(0,0,255), 1);
}

int main(int argc, char** argv){

    VideoCapture vid;
    vid.open(0);
    if (vid.isOpened()){
        cout << "VideoCapture opened" << endl;
    } else {
        cout << "Could not open video :( Exiting..." << endl;
        return 1;
    }

    SimpleBlobDetector::Params params;
    params.minThreshold = 10;
    params.maxThreshold = 200;
    params.filterByArea = true;
    params.minArea = 5000;
    params.maxArea = 10000000000;
    params.filterByCircularity = false;
    params.minCircularity = 0.5;
    params.filterByConvexity = false;
    params.minConvexity = 0.50;
    params.filterByInertia = false;
    params.minInertiaRatio = 0.01;
    params.filterByColor = true;
    params.blobColor = 255;
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

    string mode = "ready";
    Mat frame;
    Mat mask;
    Mat mask1;
    Mat mask2;
    Mat hsv;
    vector<KeyPoint> keypoints;

    while (true) {

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        vid >> frame;

        cvtColor(frame, hsv ,COLOR_BGR2HSV);
        inRange(hsv, Scalar(0, 150, 150), Scalar(10, 255, 255), mask1);
        inRange(hsv, Scalar(170, 150, 150), Scalar(180, 255, 255), mask2);
        mask = mask1 | mask2; // bitwise or instead of addition!!!

        detector->detect(mask, keypoints);
        drawKeypoints(mask,keypoints,mask);

        for (int i = 0; i < keypoints.size(); i++){
            crosshair(keypoints[i].pt.x, keypoints[i].pt.y, mask, 20);
            line(mask, Point(keypoints[i].pt.x, keypoints[i].pt.y), Point(mask.cols/2,mask.rows/2), Scalar(255,0,0), 1);
        }

        circle(mask, Point(mask.cols/2,mask.rows/2), 1, Scalar(0,0,255), 1);

        imshow("Preprocessed", frame);
        imshow("Masked", mask);

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        double time = (double) std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() / 1000000000;
        cout << fixed << setprecision(10) << time << endl;

        if ((char)cv::waitKey(10) > 0) break;
    }

    return 0;

}
