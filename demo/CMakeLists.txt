project(trakingDemo)
Cmake_minimum_required (VERSION 3.0)
FIND_PACKAGE(OpenCV 4 REQUIRED)

# for TrackingDemo
include_directories(
    ${OPENCV_INCLUDE_DIRECTORIES}
    "../include"
)
add_executable(demo  ../src/Tracking.cpp ../src/Common.cpp ../src/BatchTracker.cpp TrackingDemo.cpp)
target_link_libraries(demo ${OpenCV_LIBS})