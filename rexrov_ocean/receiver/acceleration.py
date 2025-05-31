#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
import math

class ROVAcceleration:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('rov_acceleration', anonymous=True)
        
        # 订阅IMU话题
        self.imu_sub = rospy.Subscriber('/rexrov/imu', Imu, self.imu_callback)
        
        # 初始化加速度变量
        # 线性加速度 (m/s²)
        self.linear_x = 0.0  # 前后方向
        self.linear_y = 0.0  # 左右方向
        self.linear_z = 0.0  # 上下方向
        
        # 创建定时器，每1秒显示一次数据
        self.display_timer = rospy.Timer(rospy.Duration(1), self.display_data)
        
        rospy.loginfo("加速度订阅器初始化完成")
        rospy.loginfo("等待接收IMU数据...")
    
    def imu_callback(self, msg):
        # 获取线性加速度
        self.linear_x = msg.linear_acceleration.x
        self.linear_y = msg.linear_acceleration.y
        self.linear_z = msg.linear_acceleration.z
    
    def display_data(self, event):
        rospy.loginfo("\n=== 航行器加速度数据 ===")
        rospy.loginfo("线性加速度 (m/s²):")
        rospy.loginfo("  X: %.5f", self.linear_x)
        rospy.loginfo("  Y: %.5f", self.linear_y)
        rospy.loginfo("  Z: %.5f", self.linear_z)
        rospy.loginfo("===================\n")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        acceleration_node = ROVAcceleration()
        acceleration_node.run()
    except rospy.ROSInterruptException:
        pass 