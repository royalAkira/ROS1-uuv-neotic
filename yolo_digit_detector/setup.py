#!/usr/bin/env python3

from setuptools import setup

setup(
    name='yolo_digit_detector',
    version='0.0.1',
    packages=['yolo_digit_detector'],
    package_dir={'': 'src'},
    scripts=['scripts/yolo_node.py'],
    install_requires=[
        'rospy',
        'sensor_msgs',
        'std_msgs',
        'cv_bridge',
        'torch',
        'numpy',
        'opencv-python'
    ],
    description='YOLO digit detector ROS package',
)