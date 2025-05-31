#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class WaypointNavigator:
    def __init__(self):
        rospy.init_node('waypoint_navigator')
        
        # 目标点列表
        self.waypoints = [
            Point(12.0, 10.0, -3.0),
            Point(-10.0, -8.0, -5.0)
        ]
        self.current_waypoint_index = 0
        
        # 到达阈值（米）
        self.arrival_threshold = 0.5
        
        # 订阅当前位置
        self.odom_sub = rospy.Subscriber('/rexrov/pose_gt', Odometry, self.odom_callback)
        
        # 发布速度控制命令
        self.cmd_vel_pub = rospy.Publisher('/rexrov/cmd_vel', Twist, queue_size=10)
        
        # 发布到达消息
        self.arrival_pub = rospy.Publisher('/rexrov/waypoint_arrival', String, queue_size=10)
        
        # 当前位置
        self.current_position = Point()
        
        # 控制参数
        self.linear_speed = 0.5  # 线速度
        self.angular_speed = 0.3  # 角速度
        
        rospy.loginfo("Waypoint navigator initialized")
        
    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        
    def distance_to_waypoint(self, waypoint):
        return np.sqrt(
            (self.current_position.x - waypoint.x) ** 2 +
            (self.current_position.y - waypoint.y) ** 2 +
            (self.current_position.z - waypoint.z) ** 2
        )
    
    def calculate_control_command(self, target):
        cmd = Twist()
        
        # 计算到目标点的距离
        distance = self.distance_to_waypoint(target)
        
        if distance < self.arrival_threshold:
            # 到达目标点
            arrival_msg = String()
            arrival_msg.data = f"Reached waypoint at ({target.x}, {target.y}, {target.z})"
            self.arrival_pub.publish(arrival_msg)
            rospy.loginfo(arrival_msg.data)
            
            # 移动到下一个目标点
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                rospy.loginfo("All waypoints completed!")
                return None
            return cmd
        
        # 计算方向向量
        dx = target.x - self.current_position.x
        dy = target.y - self.current_position.y
        dz = target.z - self.current_position.z
        
        # 计算水平面角度
        yaw = np.arctan2(dy, dx)
        
        # 设置线速度
        cmd.linear.x = self.linear_speed
        
        # 设置角速度（绕Z轴旋转）
        cmd.angular.z = self.angular_speed * np.sign(yaw)
        
        return cmd
    
    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            if self.current_waypoint_index < len(self.waypoints):
                current_target = self.waypoints[self.current_waypoint_index]
                cmd = self.calculate_control_command(current_target)
                
                if cmd is not None:
                    self.cmd_vel_pub.publish(cmd)
            else:
                # 所有目标点都已完成，停止移动
                cmd = Twist()
                self.cmd_vel_pub.publish(cmd)
                
            rate.sleep()

if __name__ == '__main__':
    try:
        navigator = WaypointNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass
