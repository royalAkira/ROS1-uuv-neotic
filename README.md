# ROS1-uuv-neotic
此项目主要用于解决gazebo中水下机器人仿真常用到的uuv-simulator没有专用于ROS1-neotic的版本的问题
将几个会导致报错的核心部分进行修改之后的成品代码
将这些代码替换掉原有的，即可实现在Ubuntu20.04版本中的gazebo使用uuv-simulator包，其中的sdf，urdf，xacro不需更改，mesh也无问题
for yolo_digit， you have to download yolov5 in github
for uuv_simulator， I just change the part that have to change
