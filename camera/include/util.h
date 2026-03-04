#ifndef UTIL_H
#define UTIL_H

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void draw_points(Mat& image, const vector<Point2f>& points, const Scalar& color, int radius_px, int thickness);
vector<Point2f> create_line_between_points(Point2f start, Point2f end, int num);
vector<Point2f> create_radial_line(Point2f center, double angle, double length, int num);
vector<Point2f> create_curve(Point2f start, Point2f control, Point2f end, int num);
Mat draw_text(const string& str, int width, int height);
void overlay_image(Mat& background, const Point& center, const Mat& img);

#endif //UTIL_H
