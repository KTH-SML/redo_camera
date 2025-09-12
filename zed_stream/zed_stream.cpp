#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <thread>
#include <sl/Camera.hpp>

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
bool thread_signal = false;
bool is_thread_running = false;
bool is_thread_running_2 = false;
unsigned char *d_bgra = nullptr;
unsigned char *rgb = nullptr;
unsigned char *yuyv = nullptr;
std::thread sensor_thread;
std::thread sensor_thread_2;

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
    sl::Camera zed;
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD1080; // Use HD1080 video mode
    init_params.depth_mode = sl::DEPTH_MODE::NONE;
    init_params.camera_fps = fps;
    init_params.coordinate_units = sl::UNIT::MILLIMETER;

    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS)
        exit(-1);

    sl::Mat image(1920, 1080, sl::MAT_TYPE::U8_C4);

    // Define the sensor data components
    std::unique_ptr<StreamImage> stream_image;
    std::shared_ptr<PredictionLine> prediction_line;
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
                if (is_hmi || is_p_hmi)
                {
                    vel = std::make_unique<SensorAPI>(Velocity, buffer, BUFFER_SIZE, bufferMutex);
                    ax = std::make_unique<SensorAPI>(Ax, buffer, BUFFER_SIZE, bufferMutex);
                    str_whe_phi = std::make_unique<SensorAPI>(SteeringAngle, buffer, BUFFER_SIZE, bufferMutex);
                    latency = std::make_unique<SensorAPI>(Latency, buffer_2, BUFFER_SIZE, bufferMutex_2);
                    stream_image = std::make_unique<StreamImage>(width, height);
                    prediction_line = std::make_shared<PredictionLine>("fisheye_calibration.yaml",
                                                                       "homography_calibration.yaml", width, height);
                    velocity = make_shared<TextComponent>(960, 770, 100, 100);
                    latency_label = make_shared<TextComponent>(1800, 50, 200, 100);
                    stream_image->add_component("prediction_line", std::static_pointer_cast<Component>(prediction_line));
                    stream_image->add_component("velocity", std::static_pointer_cast<Component>(velocity));
                    stream_image->add_component("latency_label", std::static_pointer_cast<Component>(latency_label));
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
            if (is_surroundings_hmi)
            {
                // Not supported yet
                
            }
            if (is_hmi || is_p_hmi)
            {
                const auto _vel = vel->get_float_value();
                const int total_delay = delay_ms + latency->get_int_value();
                if (is_p_hmi)
                {
                    prediction_line->update(_vel, ax->get_float_value(), str_whe_phi->get_float_value(), str_whe_phi->get_float_value(), total_delay / 1000.0);
                }
                else
                {
                    prediction_line->update(_vel, ax->get_float_value(), str_whe_phi->get_float_value(), str_whe_phi->get_float_value(), 0);
                }
                velocity->update(to_string(static_cast<int>(_vel)));
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

    // Cleanup
    if (is_sensor_init)
    {
        thread_signal = true;
        if (sensor_thread.joinable() && sensor_thread_2.joinable())
        {
            int count = 0;
            while ((is_thread_running || is_thread_running_2) && count++ < 30)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            if (is_thread_running || is_thread_running_2)
            {
                std::cerr << "[zed stream] Sensor thread did not exit gracefully" << std::endl;
                sensor_thread.detach();
                sensor_thread_2.detach();
            }
            else
            {
                sensor_thread.join();
                sensor_thread_2.join();
            }
        }
        thread_signal = false;
        delete[] buffer;
        buffer = nullptr;
        delete[] buffer_2;
        buffer_2 = nullptr;
        delete bridge;
        bridge = nullptr;
        delete bridge_2;
        bridge_2 = nullptr;
        is_sensor_init = false;
    }
    if (is_init)
    {
        d_bgra = nullptr;
        free_cuda_buffer(rgb);
        rgb = nullptr;
        free_cuda_buffer(yuyv);
        yuyv = nullptr;
        is_init = false;
    }
    zed.close();
    close(video_fd);
}
