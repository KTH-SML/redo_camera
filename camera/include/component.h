#ifndef COMPONENT_H
#define COMPONENT_H
#include <unordered_map>
#include <memory>
#include <opencv2/opencv.hpp>

#include "fisheye.h"
#include "homography.h"

// #define ORIGIN_X 959 // for zed camera
#define ORIGIN_X 630 // for usb camera
// #define ORIGIN_Y 1079 // for zed camera
#define ORIGIN_Y  360// for usb camera
// #define PIXELS_PER_METER 10 // for zed camera
#define PIXELS_PER_METER 10*2/3.0f // for usb camera
#define STR_WHE_RATIO 60.0f

using namespace std;
using namespace cv;

class Component;

class StreamImage
{
    unordered_map<string, shared_ptr<Component>> components;
    const int width;
    const int height;

public:
    StreamImage(int width, int height);
    void add_component(const string& name, const shared_ptr<Component>& component);
    void operator>>(const unsigned char* imageData) const;
    void operator>>(Mat& imageData) const;
};

class Component
{
public:
    virtual ~Component() = default;
    virtual void operator>>(Mat& imageData) const = 0;
};

class ImageComponent : public Component
{
protected:
    const int cx;
    const int cy;
    const int width;
    const int height;
    Mat img;
    void reset_img();

public:
    ImageComponent(int cx, int cy, int width, int height);
    void operator>>(Mat& imageData) const override;
};

class LineComponent : public Component
{
protected:
    const int width;
    const int height;
    vector<Point2f> lines_;
    cv::Scalar color_bgr_{54, 51, 226}; //BGR default blue
    int radius_px_{6};
    const Fisheye fisheye_camera;
    const Homography homography_line;
    void project(const vector<Point2f>& lines);

public:
    LineComponent(const string& fisheye_config, const string& homography_config, int width, int height, cv::Scalar color_bgr = cv::Scalar(54, 51, 226), int radius_px = 6);
    void operator>>(Mat& imageData) const override;
};

class DriverLine final : public LineComponent
{
public:
    DriverLine(const string& fisheye_config, const string& homography_config, int width, int height);
    void update(float str_whe_phi);
};

class PredictionLine final : public LineComponent
{
public:
    PredictionLine(const string& fisheye_config, const string& homography_config, int width, int height);
    void update(float v, float a, float str_whe_phi_remote, float str_whe_phi_local, float latency);
};

class PredictionSurroundingsLine final : public LineComponent
{
public:
    PredictionSurroundingsLine(const string& fisheye_config, const string& homography_config, int width, int height);
    void update(const float latency);
};

class TPComponent : public LineComponent
{
public:
    TPComponent(const string& fisheye_config, const string& homography_config, int width, int height, cv::Scalar color_bgr = cv::Scalar(223, 22, 32), int radius_px = 3);
    void update(const vector<pair<float, float>>& points);
};

class TrajectoryPoints : public LineComponent
{
    public:
    TrajectoryPoints(const string& fisheye_config, const string& homography_config, const int width, const int height, cv::Scalar color_bgr = cv::Scalar(22, 225, 225), int radius_px = 10);

    void update(const std::vector<std::pair<float, float>>& points_m);
};

class TextComponent final : public ImageComponent
{
public:
    TextComponent(int x, int y, int width, int height);
    void update(const string& text);
};

class ImageComponent_2 final : public ImageComponent
{
    const int origin_width;
    const int origin_height;
public:
    ImageComponent_2(int x, int y, int width, int height, int origin_width, int origin_height);
    void update(const unsigned char* imageData);
};

#endif //COMPONENT_H
