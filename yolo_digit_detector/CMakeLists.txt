cmake_minimum_required(VERSION 3.0.2)
project(yolo_digit_detector)

# 设置Python 3
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

#catkin_python_setup()

## Generate messages
add_message_files(
  FILES
  DetectionResult.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs std_msgs cv_bridge message_runtime
  INCLUDE_DIRS include
  LIBRARIES yolo_digit_detector
)

catkin_install_python(PROGRAMS
  scripts/yolo_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
