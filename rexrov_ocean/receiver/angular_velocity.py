#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
import math

class ROVAngularVelocity:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('rov_angular_velocity', anonymous=True)
        
        # 订阅IMU话题
        self.imu_sub = rospy.Subscriber('/rexrov/imu', Imu, self.imu_callback)
        
        # 角速度 (rad/s)
        self.angular_x = 0.0  # 横滚角速度
        self.angular_y = 0.0  # 俯仰角速度
        self.angular_z = 0.0  # 偏航角速度
        
        # 创建定时器，每1秒显示一次数据
        self.display_timer = rospy.Timer(rospy.Duration(1), self.display_data)
        
        rospy.loginfo("角速度订阅器初始化完成")
        rospy.loginfo("等待接收IMU数据...")
    
    def imu_callback(self, msg):
        # 获取角速度
        self.angular_x = msg.angular_velocity.x
        self.angular_y = msg.angular_velocity.y
        self.angular_z = msg.angular_velocity.z
    
    def display_data(self, event):
        rospy.loginfo("\n=== 航行器角速度数据 ===")
        rospy.loginfo("角速度 (rad/s):")
        rospy.loginfo("  X: %.5f", self.angular_x)
        rospy.loginfo("  Y: %.5f", self.angular_y)
        rospy.loginfo("  Z: %.5f", self.angular_z)
        rospy.loginfo("===================\n")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        angular_velocity_node = ROVAngularVelocity()
        angular_velocity_node.run()
    except rospy.ROSInterruptException:
        pass 