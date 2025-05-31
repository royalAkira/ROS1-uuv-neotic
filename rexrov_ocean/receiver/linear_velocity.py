#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Vector3
import math
import numpy as np # type: ignore

class LinearVelocityCalculator:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('rov_linear_velocity', anonymous=True)
        
        # 创建订阅者
        self.gps_sub = rospy.Subscriber('/rexrov/gps', NavSatFix, self.gps_callback)
        self.imu_sub = rospy.Subscriber('/rexrov/imu', Imu, self.imu_callback)
        
        # 初始化变量
        self.velocity = Vector3()  # 当前速度
        self.last_position = Vector3()  # 上一次位置
        self.last_time = None  # 上一次时间戳
        self.acceleration = Vector3()  # 当前加速度
        
        # 位置相关变量
        self.ref_latitude = None
        self.ref_longitude = None
        self.EARTH_RADIUS = 6378137.0
        
        # 创建定时器，每1秒显示一次速度
        self.display_timer = rospy.Timer(rospy.Duration(1), self.display_velocity)
        
        rospy.loginfo("线速度计算器初始化完成")
        rospy.loginfo("等待接收GPS和IMU数据...")

    def gps_callback(self, msg):
        """GPS消息回调函数"""
        current_time = rospy.Time.now()
        
        # 如果是第一次收到GPS数据，设置为参考点
        if self.ref_latitude is None:
            self.ref_latitude = msg.latitude
            self.ref_longitude = msg.longitude
            self.last_time = current_time
            rospy.loginfo("设置参考点：")
            rospy.loginfo("纬度: %.6f", self.ref_latitude)
            rospy.loginfo("经度: %.6f", self.ref_longitude)
            return
        
        # 计算当前位置
        current_position = Vector3()
        lat_diff = msg.latitude - self.ref_latitude
        lon_diff = msg.longitude - self.ref_longitude
        
        current_position.y = lat_diff * (math.pi * self.EARTH_RADIUS / 180.0)
        current_position.x = lon_diff * (math.pi * self.EARTH_RADIUS * math.cos(math.radians(self.ref_latitude)) / 180.0)
        current_position.z = abs(msg.altitude)
        
        # 计算时间差
        dt = (current_time - self.last_time).to_sec()
        if dt > 0:
            # 计算位置变化
            dx = current_position.x - self.last_position.x
            dy = current_position.y - self.last_position.y
            dz = current_position.z - self.last_position.z
            
            # 计算GPS速度
            gps_velocity = Vector3()
            gps_velocity.x = dx / dt
            gps_velocity.y = dy / dt
            gps_velocity.z = dz / dt
            
            # 使用IMU加速度进行速度修正
            self.velocity.x = gps_velocity.x + self.acceleration.x * dt
            self.velocity.y = gps_velocity.y + self.acceleration.y * dt
            self.velocity.z = gps_velocity.z + self.acceleration.z * dt
            
            # 更新上一次的位置和时间
            self.last_position = current_position
            self.last_time = current_time

    def imu_callback(self, msg):
        """IMU消息回调函数"""
        # 更新加速度数据
        self.acceleration = msg.linear_acceleration

    def display_velocity(self, event):
        """显示速度信息"""
        rospy.loginfo("\n=== 航行器速度信息 ===")
        rospy.loginfo("线速度:")
        rospy.loginfo("  X (东向): %.2f m/s", self.velocity.x)
        rospy.loginfo("  Y (北向): %.2f m/s", self.velocity.y)
        rospy.loginfo("  Z (垂直): %.2f m/s", self.velocity.z)
        rospy.loginfo("===================\n")

    def run(self):
        """运行节点"""
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("节点已停止")

def main():
    try:
        calculator = LinearVelocityCalculator()
        calculator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("程序正常终止")
    except Exception as e:
        rospy.logerr("程序启动失败: %s" % str(e))

if __name__ == '__main__':
    main() 