CMAKE_MINIMUM_REQUIRED(VERSION 3.5)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()

find_package(ZED REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA 12.6 REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${ZED_LIBRARY_DIR})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})

add_library(zed_stream
        STATIC
        zed_stream.cpp
)

target_include_directories(zed_stream
        PUBLIC
        .
        ${OpenCV_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/sensor/include
        ${CMAKE_SOURCE_DIR}/cuda
        ${CMAKE_SOURCE_DIR}/camera/include
)

target_link_libraries(zed_stream
        PUBLIC
        ${ZED_LIBRARIES}
        ${OpenCV_LIBS}
        ${CUDA_LIBRARIES}
        /usr/lib/aarch64-linux-gnu/libjpeg.so
        camera
        sensor
        motion
)
