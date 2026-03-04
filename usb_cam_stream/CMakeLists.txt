cmake_minimum_required(VERSION 3.10)
project(usb_cam_stream)

find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../camera/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../sensor/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../motion/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../cuda
    ${CUDA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# ライブラリとしてビルド
add_library(usb_cam_stream STATIC usb_cam_stream.cpp)

target_link_libraries(usb_cam_stream
    camera
    sensor
    motion
    cuda
    ${OpenCV_LIBS}
    ${CUDA_LIBRARIES}
)