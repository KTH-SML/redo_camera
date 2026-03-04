#include "component.h"
#include "util.h"
#include "cyra.h"
#include "kbm.h"

StreamImage::StreamImage(const int width, const int height): components({}), width(width), height(height)
{
}

void StreamImage::add_component(const string& name, const shared_ptr<Component>& component)
{
    components.emplace(name, component);
}

void StreamImage::operator>>(const unsigned char* imageData) const
{
    Mat img(height, width, CV_8UC3, const_cast<unsigned char*>(imageData));
    operator>>(img);
}

void StreamImage::operator>>(Mat& imageData) const
{
    for (const auto& [str, component] : components)
    {
        *component >> imageData;
    }
}

ImageComponent::ImageComponent(const int cx, const int cy, const int width, const int height): cx(cx), cy(cy),
    width(width),
    height(height)
{
    img = Mat::zeros(height, width, CV_8UC3);
}

void ImageComponent::reset_img()
{
    if (!img.empty() && img.size() == Size(width, height) && img.type() == CV_8UC3)
    {
        img.setTo(Scalar(0, 0, 0));
    }
    else
    {
        img.create(height, width, CV_8UC3);
        img.setTo(Scalar(0, 0, 0));
    }
}

void ImageComponent::operator>>(Mat& imageData) const
{
    overlay_image(imageData, Point(cx, cy), img);
}

LineComponent::LineComponent(const string& fisheye_config, const string& homography_config, const int width,
                             const int height, cv::Scalar color_bgr, int radius_px): width(width), height(height), lines_({}), color_bgr_(color_bgr), radius_px_(radius_px),
                                                fisheye_camera(Fisheye(fisheye_config)),
                                                homography_line(Homography(homography_config))
{
}

void LineComponent::operator>>(Mat& imageData) const
{
    draw_points(imageData, lines_, color_bgr_, radius_px_, FILLED);
}

void LineComponent::project(const vector<Point2f>& lines)
{
    homography_line.projectPoints(lines, lines_);
    // fisheye_camera.distortPoints(lines_, lines_);
}

DriverLine::DriverLine(const string& fisheye_config, const string& homography_config, const int width,
                       const int height):
    LineComponent(fisheye_config, homography_config, width, height)
{
}

void DriverLine::update(const float str_whe_phi)
{
    const float angle = str_whe_phi / STR_WHE_RATIO;
    const vector<Point2f> trajectory = kbm2(angle);
    Point2f origin = {ORIGIN_X, ORIGIN_Y};
    vector<Point2f> lines;
    lines.reserve(trajectory.size());
    for (auto point : trajectory)
    {
        lines.emplace_back(origin.x + point.x * PIXELS_PER_METER, origin.y - point.y * PIXELS_PER_METER);
    }
    project(lines);
}

PredictionLine::PredictionLine(const string& fisheye_config, const string& homography_config, const int width,
                               const int height):
    LineComponent(fisheye_config, homography_config, width, height)
{
}

void PredictionLine::update(const float v, const float a, float str_whe_phi_remote, const float str_whe_phi_local,
                            const float latency)
{
    // CYRA Model----------------------------------------------------------------------------------
    const double omega = bycicleModel(v, str_whe_phi_remote / STR_WHE_RATIO, RCVE_WHBASE, RCVE_RATIO);
    const auto [x, y, theta] = predictCYRA(v, a, omega, THETA0, latency);
    Point2f origin = {
        static_cast<float>(ORIGIN_X + y * PIXELS_PER_METER),
        static_cast<float>(ORIGIN_Y - x * PIXELS_PER_METER)
    };
    vector<Point2f> lines = create_radial_line(origin, theta, PIXELS_PER_METER, 50);
    // KBM Model-----------------------------------------------------------------------------------
    const auto cos_theta = static_cast<float>(cos(theta));
    const auto sin_theta = static_cast<float>(sin(theta));
    const float angle = str_whe_phi_local / STR_WHE_RATIO;
    const vector<Point2f> trajectory = kbm2(angle);
    vector<Point2f> lines_trajectory;
    for (auto point : trajectory)
    {
        const float x_ = origin.x + point.x * PIXELS_PER_METER;
        const float y_ = origin.y - point.y * PIXELS_PER_METER;
        const float x_r = origin.x + (x_ - origin.x) * cos_theta - (y_ - origin.y) * sin_theta;
        const float y_r = origin.y + (x_ - origin.x) * sin_theta + (y_ - origin.y) * cos_theta;
        lines_trajectory.emplace_back(x_r, y_r);
    }
    // --------------------------------------------------------------------------------------------
    lines.insert(lines.end(), lines_trajectory.begin(), lines_trajectory.end());
    project(lines);
}

PredictionSurroundingsLine::PredictionSurroundingsLine(const string& fisheye_config, const string& homography_config, const int width,
                               const int height):
    LineComponent(fisheye_config, homography_config, width, height)
{
}

void PredictionSurroundingsLine::update(const float latency)
{
    const double x = 0.0;
    const double y = 10.0; //test with 10 meters ahead
    const double v = 100.0; //test with 100 m/s speed
    const double theta = 3.14159265356*0.9; //test with 90 degree

    Point2f origin_point = {
        static_cast<float>(ORIGIN_X + x * PIXELS_PER_METER),
        static_cast<float>(ORIGIN_Y - y * PIXELS_PER_METER)
    };
    Point2f end_point = {
        static_cast<float>(ORIGIN_X + x * PIXELS_PER_METER + v * cos(theta) * latency),
        static_cast<float>(ORIGIN_Y - y * PIXELS_PER_METER - v * sin(theta) * latency)
    };
    vector<Point2f> lines = create_line_between_points(origin_point, end_point, 150);
    project(lines);
}

TPComponent::TPComponent(const string& fisheye_config, const string& homography_config, const int width,
                             const int height, cv::Scalar color_bgr, int radius_px): LineComponent(fisheye_config, homography_config, width, height, color_bgr, radius_px)
{
}

void TPComponent::update(const vector<pair<float, float>>& points)
{
    vector<Point2f> shapes;

    const float radius_m = 0.25; // 10 cm radius
    const int segments = 50; // number of segments to approximate the circle

    for (const auto& point : points)
    {
        float world_cx = point.first;
        float world_cy = point.second;

        for (int i = 0; i <= segments; ++i)
        {
            float theta = 2.0f * CV_PI * float(i) / float(segments);

            float wx = world_cx + radius_m * cos(theta);
            float wy = world_cy + radius_m * sin(theta);

            shapes.emplace_back(wx, wy);
        }
    }
    project(shapes);
    std::cout << "=== [DEBUG] TPComponent Projected Points ===" << std::endl;
    for (size_t i = 0; i < lines_.size(); ++i) {
        std::cout << "  pt[" << i << "]: (" 
                  << lines_[i].x << ", " << lines_[i].y << ")" << std::endl;
    }
    std::cout << "===============================================" << std::endl;
}


TrajectoryPoints::TrajectoryPoints(const string& fisheye_config, const string& homography_config, const int width,
                             const int height, cv::Scalar color_bgr, int radius_px): LineComponent(fisheye_config, homography_config, width, height, color_bgr, radius_px)
{
}
void TrajectoryPoints::update(const std::vector<std::pair<float, float>>& points_m)
{
    vector<Point2f> shapes;
    for (const auto& point : points_m)
    {
        float world_cx = point.first;
        float world_cy = point.second;

        shapes.emplace_back(world_cx, world_cy);
    }
    project(shapes);
    // std::cout << "=== [DEBUG] TPComponent Projected Points ===" << std::endl;
    
    // for (size_t i = 0; i < lines_.size(); ++i) {
    //     std::cout << "  pt[" << i << "]: (" 
    //               << lines_[i].x << ", " << lines_[i].y << ")" << std::endl;
    // }
    // std::cout << "===============================================" << std::endl;
}

TextComponent::TextComponent(const int x, const int y, const int width, const int height): ImageComponent(
    x, y, width, height)
{
}

void TextComponent::update(const string& text)
{
    reset_img();
    draw_text(text, width, height).copyTo(img);
}

ImageComponent_2::ImageComponent_2(const int x, const int y, const int width, const int height, const int origin_width,
                                   const int origin_height)
    : ImageComponent(x, y, width, height), origin_width(origin_width), origin_height(origin_height)
{
}

void ImageComponent_2::update(const unsigned char *imageData)
{
    reset_img();
    if (imageData)
    {
        Mat img_temp(origin_height, origin_width, CV_8UC3, const_cast<unsigned char *>(imageData));
        resize(img_temp, img, Size(width, height), 0, 0, INTER_LINEAR);
    }
    else
    {
        std::cerr << "[ImageComponent_2] Error: imageData is null" << std::endl;
    }
}
