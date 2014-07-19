cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(Kinect)

find_package( catkin REQUIRED COMPONENTS roscpp cv_bridge roscpp rospy std_msgs image_transport )
find_package( OpenCV REQUIRED )
FIND_PACKAGE(OpenCV REQUIRED)
find_package(Boost REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(kinectTest main.cpp)

target_link_libraries (kinectTest ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} /opt/ros/hydro/lib/libcv_bridge.so)