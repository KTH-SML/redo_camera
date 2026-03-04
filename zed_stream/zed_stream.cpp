#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <thread>
// #include <sl/Camera.hpp>
#include <sl/CameraOne.hpp>
#include <atomic>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <atomic>

#include <opencv2/opencv.hpp>
#include "zed_stream.h"
#include "component.h"
#include "formatting.h"
#include "socket_bridge.h"
#include "sensor.h"
#include "data_logger.h"

#define BUFFER_SIZE 2048
#define CUDA_STREAMS 8
#define FORWARD 0

#define DATA_NUM 2
const int _data_logger_ids[] = {SteeringAngle, Velocity};
const int _data_logger_type[] = {_TYPE_FLOAT, _TYPE_FLOAT};
#define DATA_NUM_2 1
const int _data_logger_ids_2[] = {Latency};
const int _data_logger_type_2[] = {_TYPE_FLOAT};

bool is_init = false;
int video_fd;
SocketBridge *bridge = nullptr;
SocketBridge *bridge_2 = nullptr;
char *buffer = nullptr;
char *buffer_2 = nullptr;
bool is_sensor_init = false;
std::atomic_bool thread_signal{false};
std::atomic_bool isRunning{false};
std::atomic_bool is_thread_running{false};
std::atomic_bool is_thread_running_2{false};
unsigned char *d_bgra = nullptr;
unsigned char *rgb = nullptr;
unsigned char *yuyv = nullptr;
std::thread sensor_thread;
std::thread sensor_thread_2;

int TP_status_sock_fd = -1;
std::thread TP_status_thread;
std::atomic<bool> TP_udp_stop{false};
std::atomic<bool> TP_udp_running{false};

std::vector<XYVV> TP_status_latest;
std::shared_mutex xyvv_mutex;

int MPC_sock_fd = -1;
std::thread MPC_thread;
std::atomic<bool> MPC_udp_stop{false};
std::atomic<bool> MPC_udp_running{false};

std::vector<std::pair<float,float>> MPC_latest_pts;
MPCPacketMeta MPC_latest_meta;
std::shared_mutex mpc_mutex;

std::shared_ptr<TPComponent> tp_visualizer;
std::shared_ptr<TrajectoryPoints> mpc_visualizer;

//[tmp for test mpc visualizing]read csv and reshape to vector<pair<float, float>>
std::vector<std::pair<float, float>> parse_mpc_trajectory(const std::string& csv_data, int horizon = 41)
{
    std::vector<float> values;
    std::stringstream ss(csv_data);
    std::string segment;

    while (std::getline(ss, segment, ',')) {
        try {
            values.push_back(std::stof(segment));
        } catch (...) {
            continue;
        }
    }

    std::vector<std::pair<float, float>> points;
    if (values.size() < horizon * 2) {
        return points;
    }

    points.reserve(horizon);

    for (int i = 0; i < horizon; ++i) {
        float x = values[i]; 
        float y = values[i + horizon];
        points.push_back({x, y});
    }

    return points;
}


void capture_frames(const char *video_device, const std::string &ip, int port, bool &signal, int fps, int delay_ms, const char *logger, bool is_hmi, bool is_p_hmi, int is_surroundings_hmi, int scale)
{
    // Open the virtual V4L2 device
    video_fd = open(video_device, O_WRONLY);
    if (video_fd < 0)
    {
        std::cerr << "[zed stream] Failed to open virtual video device" << std::endl;
        return;
    }

    // Initialize zed
    // sl::Camera zed;
    // sl::InitParameters init_params;
    // init_params.camera_resolution = sl::RESOLUTION::HD1080; // Use HD1080 video mode
    // init_params.depth_mode = sl::DEPTH_MODE::NONE;
    // init_params.camera_fps = fps;
    // init_params.coordinate_units = sl::UNIT::MILLIMETER;

    // for ZED X ONE
    sl::CameraOne zed;
    sl::InputType input;
    sl::InitParametersOne init_params;
    init_params.sdk_verbose = true;
    init_params.camera_resolution = sl::RESOLUTION::AUTO;
    init_params.camera_fps = fps;
    init_params.coordinate_units = sl::UNIT::MILLIMETER;
    input.setFromSerialNumber(307868120);
    init_params.input = input;

    auto err = zed.open(init_params);
    if (err > sl::ERROR_CODE::SUCCESS) {
        std::cerr << "[zed stream] Camera open failed: "
                << sl::toString(err) << " / " << sl::toVerbose(err) << std::endl;
        return;
    }
    auto info = zed.getCameraInformation();
    std::cout << "[zed stream] OPENED: serial=" << info.serial_number
            << " model=" << info.camera_model
            << " input=" << info.input_type
            << " res=" << info.camera_configuration.resolution.width
            << "x" << info.camera_configuration.resolution.height
            << " fps=" << info.camera_configuration.fps
            << std::endl;
    sl::Mat image;
    if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
        zed.retrieveImage(image, sl::VIEW::LEFT, sl::MEM::GPU);
    }

    static bool printed = false;
    if (!printed) {
        auto* cpu_ptr = image.getPtr<sl::uchar1>(sl::MEM::CPU);
        auto* gpu_ptr = image.getPtr<sl::uchar1>(sl::MEM::GPU);

        std::cout << "[zed stream] mat type=" << sl::toString(image.getDataType())
                << " w=" << image.getWidth() << " h=" << image.getHeight()
                << " cpu_ptr=" << static_cast<void*>(cpu_ptr)
                << " gpu_ptr=" << static_cast<void*>(gpu_ptr)
                << std::endl;
        printed = true;
    }


    // Define the sensor data components
    std::unique_ptr<StreamImage> stream_image;
    std::shared_ptr<PredictionLine> prediction_line;
    std::shared_ptr<PredictionSurroundingsLine> prediction_surroundings_line;
    std::shared_ptr<TextComponent> velocity;
    std::shared_ptr<TextComponent> latency_label;
    std::unique_ptr<SensorAPI> latency;
    std::unique_ptr<SensorAPI> vel;
    std::unique_ptr<SensorAPI> ax;
    std::unique_ptr<SensorAPI> str_whe_phi;
    std::unique_ptr<DataLogger> data_logger;
    std::unique_ptr<DataLogger> data_logger_2;
    std::shared_mutex bufferMutex;
    std::shared_mutex bufferMutex_2;

    // Initialize Streaming Component
    bool is_sensor_connected = false;
    if (port != -1)
    {
        bridge = new SocketBridge(ip, port);
        if (bridge)
        {
            is_sensor_connected = bridge->isValid();
        }
        bridge_2 = new SocketBridge(ip, port + 1);
        if (bridge_2)
        {
            is_sensor_connected = is_sensor_connected && bridge_2->isValid();
        }
    }
    if (is_sensor_connected)
    {
        std::cout << "[spinnaker stream] Listening to sensor data..." << std::endl;
    }
    else
    {
        if (bridge)
        {
            delete bridge;
            bridge = nullptr;
        }
        if (bridge_2)
        {
            delete bridge_2;
            bridge_2 = nullptr;
        }
        std::cout << "[zed stream] Sensor data not available." << std::endl;
    }

    TP_status_sock_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (TP_status_sock_fd >= 0) {
        int yes = 1;
        ::setsockopt(TP_status_sock_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
        sockaddr_in addr {};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(XYVV_PORT);
        if (::bind(TP_status_sock_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            std::cerr << "[zed stream] Failed to bind TP UDP socket" << ::close(TP_status_sock_fd);
            TP_status_sock_fd = -1;
        } else {
            TP_udp_stop = false;
            TP_status_thread = std::thread(TP_status_receive_loop,
                                           TP_status_sock_fd,
                                           std::ref(TP_udp_stop),
                                           std::ref(TP_udp_running),
                                           std::ref(TP_status_latest),
                                           std::ref(xyvv_mutex));
            std::cout << "[zed stream] TP UDP socket initialized on port " << XYVV_PORT << std::endl;

        }
    } else {
        std::cerr << "[zed stream] Failed to create TP UDP socket" << std::endl;
    }

    // MPC UDP socket initialization
    MPC_sock_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (MPC_sock_fd >= 0) {
        int yes = 1;
        ::setsockopt(MPC_sock_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(MPC_PORT);

        if (::bind(MPC_sock_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            std::cerr << "[usb stream] Failed to bind MPC UDP socket\n";
            ::close(MPC_sock_fd);
            MPC_sock_fd = -1;
        } else {
            MPC_udp_stop = false;
            MPC_thread = std::thread(MPC_receive_loop,
                                    MPC_sock_fd,
                                    std::ref(MPC_udp_stop),
                                    std::ref(MPC_udp_running),
                                    std::ref(MPC_latest_pts),
                                    std::ref(MPC_latest_meta),
                                    std::ref(mpc_mutex));
            std::cout << "[usb stream] MPC UDP socket initialized on port " << MPC_PORT << std::endl;
        }
    } else {
        std::cerr << "[usb stream] Failed to create MPC UDP socket\n";
    }

    // Define the converter pointer
    std::unique_ptr<CudaImageConverter> converter_bgra2rgb;
    std::unique_ptr<CudaImageConverter> converter_rgb2yuyv;

    unsigned int width;
    unsigned int height;

    while (!signal)
    {
        // Grab an image
        if (zed.grab() == sl::ERROR_CODE::SUCCESS)
        {
            zed.retrieveImage(image, sl::VIEW::LEFT, sl::MEM::GPU);
            // zed.retrieveImage(image, sl::MEM::GPU);   // view/mem指定なし
        }

        // Print the pixel format only once and initialize the virtual device and CUDA
        if (!is_init)
        {
            // Get the image size
            width = image.getWidth();
            height = image.getHeight();
            std::cout << "[zed stream] Image size: " << width << "x" << height << std::endl;

            // Set the video device format
            struct v4l2_format vfmt = {};
            vfmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
            vfmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
            vfmt.fmt.pix.field = V4L2_FIELD_NONE;
            vfmt.fmt.pix.width = width;
            vfmt.fmt.pix.height = height;
            vfmt.fmt.pix.bytesperline = width * 2;
            vfmt.fmt.pix.sizeimage = width * height * 2;
            if (ioctl(video_fd, VIDIOC_S_FMT, &vfmt) < 0)
            {
                std::cerr << "[zed stream] Failed to set video format on virtual device" << std::endl;
                break;
            }

            // Set the frame rate
            struct v4l2_streamparm streamparm = {};
            streamparm.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
            streamparm.parm.output.timeperframe.numerator = 1;
            streamparm.parm.output.timeperframe.denominator = fps;
            if (ioctl(video_fd, VIDIOC_S_PARM, &streamparm) < 0)
            {
                std::cerr << "[zed stream] Failed to set frame rate" << std::endl;
                break;
            }

            // Initialize CUDA and allocate memory for
            converter_bgra2rgb = std::make_unique<CudaImageConverter>(width, height, CUDA_STREAMS, D_BGRA2RGB);
            converter_rgb2yuyv = std::make_unique<CudaImageConverter>(width, height, CUDA_STREAMS, RGB2YUYV);
            rgb = get_cuda_buffer(width * height * 3);
            yuyv = get_cuda_buffer(width * height * 2);

            std::cout << "[zed stream] Converting to YUYV422 format..." << std::endl;
            is_init = true;
        }

        d_bgra = image.getPtr<sl::uchar1>(sl::MEM::GPU);
        converter_bgra2rgb->convert(d_bgra, rgb);

        // Add components to the image
        if (is_sensor_connected)
        {
            if (!is_sensor_init)
            {
                buffer = new char[BUFFER_SIZE]();
                buffer_2 = new char[BUFFER_SIZE]();
                if (is_hmi || is_p_hmi || is_surroundings_hmi)
                {
                    vel = std::make_unique<SensorAPI>(Velocity, buffer, BUFFER_SIZE, bufferMutex);
                    ax = std::make_unique<SensorAPI>(Ax, buffer, BUFFER_SIZE, bufferMutex);
                    str_whe_phi = std::make_unique<SensorAPI>(SteeringAngle, buffer, BUFFER_SIZE, bufferMutex);
                    latency = std::make_unique<SensorAPI>(Latency, buffer_2, BUFFER_SIZE, bufferMutex_2);
                    stream_image = std::make_unique<StreamImage>(width, height);
                    prediction_line = std::make_shared<PredictionLine>("fisheye_calibration.yaml",
                                                                       "homography_calibration.yaml", width, height);
                    prediction_surroundings_line = std::make_shared<PredictionSurroundingsLine>(
                        "fisheye_calibration.yaml", "homography_calibration.yaml", width, height);
                    
                    // velocity = make_shared<TextComponent>(960, 770, 100, 100); //for zed stereo left
                    velocity = make_shared<TextComponent>(1800, 1100, 300, 200);
                    latency_label = make_shared<TextComponent>(1800, 50, 300, 200);
                    stream_image->add_component("prediction_line", std::static_pointer_cast<Component>(prediction_line));
                    stream_image->add_component("prediction_surroundings_line", std::static_pointer_cast<Component>(prediction_surroundings_line));
                    stream_image->add_component("velocity", std::static_pointer_cast<Component>(velocity));
                    stream_image->add_component("latency_label", std::static_pointer_cast<Component>(latency_label));

                    tp_visualizer = std::make_shared<TPComponent>(
                        "fisheye_calibration.yaml", "homography_calibration_world_to_Image.yaml", width, height, Scalar(223, 22, 32), 6);
                    stream_image->add_component("tp_visualizer", std::static_pointer_cast<Component>(tp_visualizer));

                    mpc_visualizer = std::make_shared<TrajectoryPoints>(
                        "fisheye_calibration.yaml", "homography_calibration_world_to_Image.yaml", width, height, Scalar(32, 220, 223), 10);
                    stream_image->add_component("mpc_visualizer", std::static_pointer_cast<Component>(mpc_visualizer));
                }
                if (logger)
                {
                    data_logger = std::make_unique<DataLogger>(_data_logger_ids, _data_logger_type, DATA_NUM, buffer, BUFFER_SIZE, bufferMutex, logger);
                    std::string logger_2 = std::string(logger);
                    size_t pos = logger_2.find(".csv");
                    if (pos != std::string::npos)
                    {
                        logger_2.insert(pos, "_2");
                    }
                    data_logger_2 = std::make_unique<DataLogger>(_data_logger_ids_2, _data_logger_type_2, DATA_NUM_2, buffer_2, BUFFER_SIZE, bufferMutex_2, logger_2.c_str());
                }
                sensor_thread = std::thread(receive_data_loop, bridge, buffer, BUFFER_SIZE, std::ref(bufferMutex),
                                            std::ref(thread_signal), std::ref(is_thread_running));
                sensor_thread_2 = std::thread(receive_data_loop, bridge_2, buffer_2, BUFFER_SIZE, std::ref(bufferMutex_2),
                                              std::ref(thread_signal), std::ref(is_thread_running_2));
                is_sensor_init = true;
            }
            if (is_hmi || is_p_hmi || is_surroundings_hmi)
            {
                const auto _vel = vel->get_float_value();
                const int total_delay = delay_ms + latency->get_int_value();
                std::vector<XYVV> TP_status_copy;
                {
                    std::shared_lock lock(xyvv_mutex);
                    TP_status_copy = TP_status_latest;
                }
                std::vector<std::pair<float, float>> points_for_display;
                points_for_display.reserve(TP_status_copy.size());

                // std::cout << "TP count: " << TP_status_copy.size() << std::endl;
                for (size_t i = 0; i < TP_status_copy.size(); ++i) {
                    // std::cout << "TP " << i << ": x=" << TP_status_copy[i].x
                    //           << " y=" << TP_status_copy[i].y
                    //           << " vx=" << TP_status_copy[i].vx
                    //           << " vy=" << TP_status_copy[i].vy << std::endl;
                    // points_for_display.emplace_back(TP_status_copy[i].x, TP_status_copy[i].y);
                    points_for_display.emplace_back(TP_status_copy[i].x, 1.0 * TP_status_copy[i].y);
                }

                std::vector<std::pair<float,float>> mpc_pts_copy;
                MPCPacketMeta meta_copy;
                {
                    std::shared_lock lock(mpc_mutex);
                    mpc_pts_copy = MPC_latest_pts;
                    meta_copy = MPC_latest_meta;
                }
                tp_visualizer->update(points_for_display);

                if(!mpc_pts_copy.empty()){
                    for(auto& pt : mpc_pts_copy){
                        pt.second *= 1.0f;  // if needed
                    }
                    mpc_visualizer->update(mpc_pts_copy);

                }
                
                if (is_p_hmi)
                {
                    prediction_line->update(_vel, ax->get_float_value(), str_whe_phi->get_float_value(), str_whe_phi->get_float_value(), total_delay / 1000.0);
                }
                else
                {
                    prediction_line->update(_vel, ax->get_float_value(), str_whe_phi->get_float_value(), str_whe_phi->get_float_value(), 0);
                }
                if (is_surroundings_hmi)
                {
                    prediction_surroundings_line->update(total_delay / 1000.0);
                }

                velocity->update(to_string(static_cast<int>(_vel)));

                std::ostringstream velocity_ss;
                velocity_ss << std::fixed << std::setprecision(2) << _vel << " m/s";
                velocity->update(velocity_ss.str());
                latency_label->update(std::to_string(total_delay) + " ms");
                *stream_image >> rgb;
            }
            if (logger)
            {
                data_logger->logger();
                data_logger_2->logger();
            }
        }
        converter_rgb2yuyv->convert(rgb, yuyv);

        // Write the YUYV422 (16 bits per pixel) data to the virtual video device as YUYV422
        if (write(video_fd, yuyv, width * height * 2) == -1)
        {
            std::cerr << "[zed stream] Error writing frame to virtual device" << std::endl;
            break;
        }
    }

// Cleanup (reliable, no detach)
auto join_if_joinable = [](std::thread& t, const char* name) {
    if (!t.joinable()) return;
    try {
        t.join();
    } catch (...) {
        std::cerr << "[cleanup] join threw exception: " << name << std::endl;
    }
};

auto close_udp_socket = [](int& fd) {
    if (fd >= 0) {
        ::shutdown(fd, SHUT_RDWR);
        ::close(fd);
        fd = -1;
    }
};

if (is_sensor_init) {
    thread_signal = true;

    if (bridge)   bridge->shutdownAndClose();
    if (bridge_2) bridge_2->shutdownAndClose();

    join_if_joinable(sensor_thread,   "sensor_thread");
    join_if_joinable(sensor_thread_2, "sensor_thread_2");

    thread_signal = false;

    delete[] buffer;   buffer   = nullptr;
    delete[] buffer_2; buffer_2 = nullptr;

    delete bridge;   bridge   = nullptr;
    delete bridge_2; bridge_2 = nullptr;

    is_sensor_init = false;
}

if (TP_status_sock_fd >= 0) {
    TP_udp_stop.store(true, std::memory_order_release);
    close_udp_socket(TP_status_sock_fd);
    join_if_joinable(TP_status_thread, "TP_status_thread");
}

if (MPC_sock_fd >= 0) {
    MPC_udp_stop.store(true, std::memory_order_release);
    close_udp_socket(MPC_sock_fd);
    join_if_joinable(MPC_thread, "MPC_thread");
}

if (is_init) {
    d_bgra = nullptr;

    if (rgb)  { free_cuda_buffer(rgb);  rgb  = nullptr; }
    if (yuyv) { free_cuda_buffer(yuyv); yuyv = nullptr; }

    is_init = false;
}

zed.close();

if (video_fd >= 0) {
    ::close(video_fd);
    video_fd = -1;
}
}
