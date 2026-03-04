#ifndef USB_CAM_STREAM_H
#define USB_CAM_STREAM_H

#include <string>

void run_usb_cam_stream(const char *videoDevice, const char *ip, int port, int fps, int delay_ms, const char *logger, const bool is_hmi, const bool is_p_hmi, const int is_surroundings_hmi, const int scale);

void capture_usb_frames(const char *video_device, const std::string &ip, int port, bool &signal, int fps, int delay_ms, const char *logger, bool is_hmi, bool is_p_hmi, int is_surroundings_hmi, int scale);

#endif