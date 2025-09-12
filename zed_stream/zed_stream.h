#ifndef ZED_STREAM_H
#define ZED_STREAM_H

/**
 * Capture frames from ZED camera and stream them to a virtual V4L2 device.
 * 
 * @param video_device Path to the virtual V4L2 device (e.g., /dev/video16)
 * @param ip IP address of the sensor data source
 * @param port Port number of the sensor data source
 * @param signal Reference to a boolean signal to stop the streaming
 * @param fps Frames per second for the ZED camera
 * @param delay_ms Delay in milliseconds between frames (Not supported yet)
 * @param logger Path to the log file for sensor data (optional)
 * @param is_hmi Flag to indicate if HMI data should be processed
 * @param is_p_hmi Flag to indicate if P-HMI data should be processed
 * @param is_surroundings_hmi Flag to indicate if surroundings(othre traffic participants) HMI data should be processed (Not supported yet)
 * @param scale Scale factor for the output video (Not supported yet)
 */
void capture_frames(const char* video_device, const std::string& ip, int port, bool &signal, int fps, int delay_ms, const char* logger, bool is_hmi, bool is_p_hmi, int is_surroundings_hmi, int scale);

#endif // ZED_STREAM_H