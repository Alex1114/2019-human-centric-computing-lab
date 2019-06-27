cmake_minimum_required(VERSION 2.8.3)
project(lab1)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(undistort src/undistort_img.cpp)

 target_link_libraries(undistort
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )
