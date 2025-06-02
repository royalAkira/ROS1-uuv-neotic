#!/bin/bash

# 激活虚拟环境
source ~/yolo_env/bin/activate

# 设置ROS环境
source ~/catkin_ws/devel/setup.bash

# 启动节点
roslaunch yolo_digit_detector yolo_detector.launch 