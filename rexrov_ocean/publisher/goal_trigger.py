#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

class GoalTriggerController:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('goal_trigger_controller', anonymous=True)
        
        # 创建发布者
        self.goal_pub = rospy.Publisher('/rexrov/goal', Point, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/rexrov/cmd_vel', Twist, queue_size=10)
        
        # 订阅者
        self.pose_sub = rospy.Subscriber('/rexrov/pose_gt', Odometry, self.pose_callback)
        self.trigger_sub = rospy.Subscriber('/yolov5/detection_status', Bool, self.trigger_callback)
        
        # 控制器参数
        self.kp_xy = 0.3  # XY平面位置控制增益
        self.kp_z = 0.2   # 深度控制增益
        self.kd_xy = 0.1  # XY平面速度控制增益
        self.kd_z = 0.1   # 深度速度控制增益
        
        # 状态变量
        self.current_pose = None
        self.current_velocity = None
        self.trigger_received = False
        self.waiting_for_detection = False
        
        # 设置航点列表
        self.waypoints = [
            Point(10.0, 20.0, -5.0),
            Point(-10.0, -15.0, -8.0),
        ]
        self.current_waypoint_index = 0
        self.goal_position = self.waypoints[self.current_waypoint_index]
        
        # 发布目标点
        self.goal_pub.publish(self.goal_position)
        rospy.loginfo("当前目标点 %d: x=%.2f, y=%.2f, z=%.3f", 
                     self.current_waypoint_index + 1,
                     self.goal_position.x, self.goal_position.y, self.goal_position.z)
        
        rospy.loginfo("控制器初始化完成")

    def check_waypoint_reached(self):
        """检查是否到达当前航点"""
        if self.current_pose is None:
            return False
            
        # 计算到目标点的距离
        dx = self.goal_position.x - self.current_pose.position.x
        dy = self.goal_position.y - self.current_pose.position.y
        dz = self.goal_position.z - self.current_pose.position.z
        
        # 分别检查各个方向的距离
        xy_distance = np.sqrt(dx*dx + dy*dy)
        z_distance = abs(dz)
        
        # 如果水平距离小于1米且垂直距离小于0.5米，认为到达目标点
        return xy_distance < 1.0 and z_distance < 0.5

    def pose_callback(self, msg):
        """处理位置和姿态数据"""
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist

    def trigger_callback(self, msg):
        """处理触发信号"""
        if msg.data and self.waiting_for_detection:  # 如果收到True信号且正在等待检测
            self.trigger_received = True
            self.waiting_for_detection = False
            rospy.loginfo("目标识别成功，准备前往下一个目标点")

    def compute_control(self):
        """计算控制输出"""
        if self.current_pose is None or self.current_velocity is None:
            return None

        # 创建控制命令
        cmd = Twist()
        
        # 如果正在等待检测，保持静止
        if self.waiting_for_detection:
            return cmd
        
        # 计算位置误差
        dx = self.goal_position.x - self.current_pose.position.x
        dy = self.goal_position.y - self.current_pose.position.y
        dz = self.goal_position.z - self.current_pose.position.z
        
        # 设置误差阈值
        error_threshold = 0.5
        
        # 根据距离动态调整速度
        max_vel = 1.0
        
        # 计算目标速度（使用比例控制）
        target_vel_x = self.kp_xy * dx if abs(dx) > error_threshold else 0.0
        target_vel_y = self.kp_xy * dy if abs(dy) > error_threshold else 0.0
        target_vel_z = self.kp_z * dz if abs(dz) > error_threshold else 0.0
        
        # 限制目标速度
        target_vel_x = np.clip(target_vel_x, -max_vel, max_vel)
        target_vel_y = np.clip(target_vel_y, -max_vel, max_vel)
        target_vel_z = np.clip(target_vel_z, -max_vel, max_vel)
        
        # 计算速度误差
        dvx = target_vel_x - self.current_velocity.linear.x
        dvy = target_vel_y - self.current_velocity.linear.y
        dvz = target_vel_z - self.current_velocity.linear.z
        
        # 计算控制输出（加入速度反馈）
        cmd.linear.x = target_vel_x + self.kd_xy * dvx
        cmd.linear.y = target_vel_y + self.kd_xy * dvy
        cmd.linear.z = target_vel_z + self.kd_z * dvz
        
        # 限制最终控制输出
        cmd.linear.x = np.clip(cmd.linear.x, -max_vel, max_vel)
        cmd.linear.y = np.clip(cmd.linear.y, -max_vel, max_vel)
        cmd.linear.z = np.clip(cmd.linear.z, -max_vel, max_vel)
        
        return cmd

    def run(self):
        """运行控制器"""
        rate = rospy.Rate(10)  # 10Hz控制频率
        
        rospy.loginfo("开始航行...")
        
        while not rospy.is_shutdown():
            cmd = self.compute_control()
            if cmd is not None:
                # 检查是否到达当前航点
                if self.check_waypoint_reached() and not self.waiting_for_detection and not self.trigger_received:
                    rospy.loginfo("已到达航点 %d，位置: x=%.2f, y=%.2f, z=%.3f", 
                                self.current_waypoint_index + 1,
                                self.current_pose.position.x,
                                self.current_pose.position.y,
                                self.current_pose.position.z)
                    rospy.loginfo("等待目标识别...")
                    self.waiting_for_detection = True
                    # 发送零速度命令，确保停止
                    stop_cmd = Twist()
                    self.cmd_vel_pub.publish(stop_cmd)
                    continue  # 跳过本次循环，保持静止
                
                # 如果正在等待检测，保持静止
                if self.waiting_for_detection:
                    stop_cmd = Twist()
                    self.cmd_vel_pub.publish(stop_cmd)
                    continue  # 跳过本次循环，保持静止
                
                # 如果收到触发信号，前往下一个航点
                if self.trigger_received:
                    # 更新到下一个航点
                    self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
                    self.goal_position = self.waypoints[self.current_waypoint_index]
                    self.goal_pub.publish(self.goal_position)
                    rospy.loginfo("前往新目标点 %d: x=%.2f, y=%.2f, z=%.3f", 
                                self.current_waypoint_index + 1,
                                self.goal_position.x, self.goal_position.y, self.goal_position.z)
                    self.trigger_received = False
                    self.waiting_for_detection = False
                
                # 正常航行控制
                self.cmd_vel_pub.publish(cmd)
                
            rate.sleep()

def main():
    try:
        controller = GoalTriggerController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("控制器已停止")
        # 发送零速度命令
        cmd = Twist()
        controller.cmd_vel_pub.publish(cmd)
        rospy.loginfo("已发送零速度命令")
    except Exception as e:
        rospy.logerr("控制器运行出错: %s", str(e))
        # 发送零速度命令
        cmd = Twist()
        controller.cmd_vel_pub.publish(cmd)
        rospy.loginfo("已发送零速度命令")

if __name__ == '__main__':
    main() 