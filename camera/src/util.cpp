#include <opencv2/opencv.hpp>

#include "util.h"

using namespace std;
using namespace cv;

void operator+=(Mat& lhs, const vector<Point2f>& rhs)
{
    for (const auto& p : rhs)
    {
        circle(lhs, p, 6, Scalar(54, 51, 226), FILLED);
    }
}

vector<Point2f> create_line_between_points(Point2f start, Point2f end, int num)
{
    vector<Point2f> line;
    line.reserve(num);
    for (int i = 0; i < num; i++)
    {
        line.push_back(start + (end - start) * i / num);
    }
    return line;
}

vector<Point2f> create_radial_line(Point2f center, double angle, double length, int half_num)
{
    vector<Point2f> line;
    line.reserve(half_num * 2);
    for (int i = 0; i < half_num; i++)
    {
        double x_l = center.x - length * cos(angle) * i / half_num;
        double y_l = center.y - length * sin(angle) * i / half_num;
        double x_r = center.x + length * cos(angle) * i / half_num;
        double y_r = center.y + length * sin(angle) * i / half_num;
        line.emplace_back(x_l, y_l);
        line.emplace_back(x_r, y_r);
    }
    return line;
}


vector<Point2f> create_curve(Point2f start, Point2f control, Point2f end, int num)
{
    vector<Point2f> curve;
    curve.reserve(num);
    for (int i = 0; i <= num; i++)
    {
        float t = static_cast<float>(i) / static_cast<float>(num);
        float u = 1 - t;
        Point2f point = u * u * start + 2 * u * t * control + t * t * end;
        curve.push_back(point);
    }
    return curve;
}

Mat draw_text(const string& str, const int width, const int height)
{
    Mat image = Mat::zeros(height, width, CV_8UC3);
    double scale = height / 150.0;
    int thickness = max(1, height / 20);
    int baseline = 0;
    Size text_size = getTextSize(str, FONT_HERSHEY_SIMPLEX, scale, thickness, &baseline);
    Point text_origin((width - text_size.width) / 2, (height + text_size.height) / 2);
    putText(image, str, text_origin, FONT_HERSHEY_SIMPLEX, scale, Scalar(255, 0, 0), thickness);
    return image;
}

void overlay_image(Mat& background, const Point& center, const Mat& img)
{
    Mat gray, mask;
    cvtColor(img, gray, COLOR_BGR2GRAY);
    threshold(gray, mask, 1, 255, THRESH_BINARY);
    int x = center.x - img.cols / 2;
    int y = center.y - img.rows / 2;
    x = max(0, min(x, background.cols - img.cols));
    y = max(0, min(y, background.rows - img.rows));
    Rect roi(x, y, img.cols, img.rows);
    img.copyTo(background(roi), mask);
}
