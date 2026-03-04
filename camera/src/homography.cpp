#include <opencv2/opencv.hpp>

#include"homography.h"

using namespace std;
using namespace cv;

Homography::Homography(const vector<Point2f> *srcPoints, const vector<Point2f> *dstPoints)
{
    if (srcPoints->size() != dstPoints->size() || srcPoints->size() < 4)
    {
        cerr << "[homography] There must be at least 4 points and the number of source and destination points must be the same." << endl;
        return;
    }
    H = findHomography(*srcPoints, *dstPoints, RANSAC);
}

Homography::Homography(const String &filename)
{
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
    {
        cerr << "[homography] Cannot open file " << filename << endl;
        return;
    }
    fs["H"] >> H;
    fs.release();
}

void Homography::save(const String &filename) const
{
    FileStorage fs(filename, FileStorage::WRITE);
    fs << "H" << H;
    fs.release();
}

void Homography::projectPoints(const vector<Point2f> &srcPoints, vector<Point2f> &dstPoints) const
{
    if (H.empty()) {
        cerr << "[homography] Matrix H is empty!" << endl;
        return;
    }
    if (srcPoints.empty()) {
        dstPoints.clear();
        return;
    }

    try {
        cv::perspectiveTransform(srcPoints, dstPoints, H);
    } 
    catch (const cv::Exception& e) {
        cerr << "[homography] Error in perspectiveTransform: " << e.what() << endl;
        cerr << "  srcPoints size: " << srcPoints.size() << endl;
        cerr << "  H size: " << H.rows << "x" << H.cols << ", type: " << H.type() << endl;
    }
}