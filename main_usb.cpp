#include <iostream>
#include <unordered_map>
#include <string>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <csignal>

// #include "zed_stream.h"
#include "usb_cam_stream/usb_cam_stream.h"
#include "fisheye.h"
#include "homography.h"

#include <fstream>

#define DEFAULT_VIDEO_DEVICE "/dev/video16"
#define DEFAULT_IP "0.0.0.0"
#define DEFAULT_PORT 10086
#define DEFAULT_FPS 30
#define DEFAULT_DELAY_MS 0
#define DEFAULT_SCALE 1

bool capture_signal = false;

void run_zed_stream(const char *videoDevice, const char *ip, int port, int fps, int delay_ms, const char *logger, const bool is_hmi, const bool is_p_hmi, const bool is_surroundings_hmi, const int scale)
{
    std::cout << "[main] Starting to capture frames from the ZED camera..." << std::endl;
    capture_usb_frames(videoDevice, ip, port, capture_signal, fps, delay_ms, logger, is_hmi, is_p_hmi, is_surroundings_hmi, scale);
    std::cout << "[main] Capture process finished" << std::endl;
}

void run_fc()
{
    const Size boardSize(10, 7);
    constexpr float squareSize = 0.025f;
    const String filename = "data/*.jpg";
    const Fisheye fisheye(boardSize, squareSize, &filename);
    fisheye.save("fisheye_calibration.yaml");
}

void run_fu(const std::string &filename)
{
    const Fisheye camera("fisheye_calibration.yaml");
    Mat image = imread(filename);
    camera.undistort(image, image);
    imwrite("undistorted.jpg", image);
}

void run_hc()
{
    const vector<Point2f> src{{949, 1079}, {969, 1079}, {949, 579}, {969, 579}};
    vector<Point2f> dst;
    FileStorage fs("homography_points.yaml", FileStorage::READ);
    fs["points"] >> dst;
    const Homography homography(&src, &dst);
    homography.save("homography_calibration.yaml");
}
void signalHandler(int signum)
{
    std::cout << "[main] Received signal " << signum << ", cleaning up resources..." << std::endl;
    capture_signal = true;
}

std::unordered_map<std::string, std::string> parseArguments(int argc, char *argv[])
{
    std::unordered_map<std::string, std::string> args;

    for (int i = 1; i < argc; ++i)
    {
        std::string key = argv[i];
        if (key.rfind('-', 0) == 0)
        {
            if (i + 1 < argc && argv[i + 1][0] != '-')
            {
                args[key] = argv[i + 1];
                ++i;
            }
            else
            {
                args[key] = "";
            }
        }
    }

    return args;
}

int main(int argc, char *argv[])
{
    auto args = parseArguments(argc, argv);

    if (args.find("-h") != args.end())
    {
        std::cout << "[main] Usage: " << argv[0] << " [-d <video_device>] [-s [-ip <ip>] [-p <port>] [-hmi]]" << std::endl;
        std::cout << "[main] Options:" << std::endl;
        std::cout << "[main]   -d <video_device>    Specify the video device to capture frames from (default: /dev/video16)" << std::endl;
        std::cout << "[main]   -fps <fps>           Specify the frames per second (default: 60)" << std::endl;
        std::cout << "[main]   -scale <scale>       Specify the scale of the frame size (default: 1)" << std::endl;
        std::cout << "[main]   -delay <ms>          Specify the delay in milliseconds for video output (default: 0)" << std::endl;
        std::cout << "[main]   -s                   Add the sensor data to the video stream" << std::endl;
        std::cout << "[main]   -hmi                 Add HMI to the stream" << std::endl;
        std::cout << "[main]   -p_hmi               Add Prediction HMI to the stream" << std::endl;
        std::cout << "[main]   -surroundings_hmi    Add Surroundings HMI to the stream" << std::endl;
        std::cout << "[main]   -ip <ip>             Specify the IP address to stream frames to (default: 0.0.0.0)" << std::endl;
        std::cout << "[main]   -p <port>            Specify the port to stream frames to (default: 10086)" << std::endl;
        std::cout << "[main]   -log <logger_file>   Specify the logger file to log sensor data" << std::endl;
        std::cout << "[main]   -fc                  Run fisheye calibration" << std::endl;
        std::cout << "[main]   -fu <image>          Run fisheye undistortion on the specified image" << std::endl;
        std::cout << "[main]   -hc                  Run homography calibration" << std::endl;
        return 0;
    }

    if (args.find("-fc") != args.end())
    {
        run_fc();
        return 0;
    }

    if (args.find("-fu") != args.end())
    {
        const std::string filename = args["-fu"];
        if (filename.empty())
        {
            std::cerr << "[main] Usage: " << argv[0] << " -fu <image>" << std::endl;
            return -1;
        }
        run_fu(args["-fu"]);
        return 0;
    }

    if (args.find("-hc") != args.end())
    {
        run_hc();
        return 0;
    }

    const char *videoDevice;
    if (args.find("-d") != args.end())
    {
        videoDevice = args["-d"].c_str();
    }
    else
    {
        videoDevice = DEFAULT_VIDEO_DEVICE;
    }
    std::cout << "[main] Using video device: " << videoDevice << std::endl;

    const char *ip;
    int port;
    if (args.find("-s") != args.end())
    {
        if (args.find("-ip") != args.end())
        {
            ip = args["-ip"].c_str();
        }
        else
        {
            ip = DEFAULT_IP;
        }
        if (args.find("-p") != args.end())
        {
            port = std::stoi(args["-p"]);
        }
        else
        {
            port = DEFAULT_PORT;
        }
        std::cout << "[main] Using IP: " << ip << " and port: " << port << "/" << port + 1 << std::endl;
    }
    else
    {
        ip = "0.0.0.0";
        port = -1;
    }

    int fps;
    if (args.find("-fps") != args.end())
    {
        fps = std::stoi(args["-fps"]);
    }
    else
    {
        fps = DEFAULT_FPS;
    }
    std::cout << "[main] Using FPS: " << fps << std::endl;

    int delay_ms;
    if (args.find("-delay") != args.end())
    {
        delay_ms = std::stoi(args["-delay"]);
    }
    else
    {
        delay_ms = DEFAULT_DELAY_MS;
    }
    std::cout << "[main] Using delay: " << delay_ms << "ms" << std::endl;

    std::string logger_filename;
    const char *logger;
    if (args.find("-log") != args.end())
    {
        const auto file_name = args["-log"];
        auto now = std::time(nullptr);
        auto tm = *std::localtime(&now);
        std::ostringstream timestamp;
        timestamp << std::put_time(&tm, "%Y%m%d_%H%M%S");
        logger_filename = file_name + "_" + timestamp.str() + ".csv";
        logger = logger_filename.c_str();
        std::cout << "[main] Using logger file: " << logger << std::endl;
    }
    else
    {
        logger = nullptr;
    }

    bool is_hmi;
    bool is_p_hmi;
    bool is_surroundings_hmi;
    if (args.find("-hmi") != args.end())
    {
        is_hmi = true;
        std::cout << "[main] HMI mode enabled" << std::endl;
    }
    else
    {
        is_hmi = false;
        std::cout << "[main] HMI mode disabled" << std::endl;
    }

    if (args.find("-p_hmi") != args.end())
    {
        is_p_hmi = true;
        std::cout << "[main] P-HMI mode enabled" << std::endl;
    }
    else
    {
        is_p_hmi = false;
        std::cout << "[main] P-HMI mode disabled" << std::endl;
    }

    if (args.find("-surroundings_hmi") != args.end())
    {
        is_surroundings_hmi = true;
        std::cout << "[main] Surroundings HMI mode enabled" << std::endl;
    }
    else
    {
        is_surroundings_hmi = false;
        std::cout << "[main] Surroundings HMI mode disabled" << std::endl;
    }

    int scale;
    if (args.find("-scale") != args.end())
    {
        scale = std::stoi(args["-scale"]);
    }
    else
    {
        scale = DEFAULT_SCALE;
    }
    std::cout << "[main] Using scale: " << scale << std::endl;

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    run_usb_cam_stream(videoDevice, ip, port, fps, delay_ms, logger, is_hmi, is_p_hmi, is_surroundings_hmi,scale);
    return 0;
}
