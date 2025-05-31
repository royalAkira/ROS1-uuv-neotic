#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import math

class ROVPosture:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('rov_posture', anonymous=True)
        
        # 订阅IMU话题
        self.imu_sub = rospy.Subscriber('/rexrov/imu', Imu, self.imu_callback)
        
        # 初始化姿态变量
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # 创建定时器，每1秒显示一次姿态
        self.display_timer = rospy.Timer(rospy.Duration(1), self.display_posture)
        
        rospy.loginfo("IMU姿态订阅器初始化完成")
        rospy.loginfo("等待接收IMU数据...")
    
    def imu_callback(self, msg):
        # 将四元数转换为欧拉角
        orientation = msg.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        self.roll, self.pitch, self.yaw = euler_from_quaternion(quaternion)
        
        # 将弧度转换为度数，便于显示
        self.roll = math.degrees(self.roll)
        self.pitch = math.degrees(self.pitch)
        self.yaw = math.degrees(self.yaw)
    
    def display_posture(self, event):
        rospy.loginfo("\n=== 航行器姿态信息 ===")
        rospy.loginfo("Roll:  %.5f°", self.roll)
        rospy.loginfo("Pitch: %.5f°", self.pitch)
        rospy.loginfo("Yaw:   %.5f°", self.yaw)
        rospy.loginfo("===================\n")
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        posture_node = ROVPosture()
        posture_node.run()
    except rospy.ROSInterruptException:
        pass
