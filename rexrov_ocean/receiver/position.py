#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry

class PositionSubscriber:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('rov_position_subscriber', anonymous=True)
        
        # 创建订阅者
        self.pose_sub = rospy.Subscriber('/rexrov/pose_gt', Odometry, self.pose_callback)
        
        # 初始化位置信息
        self.position = Vector3()
        
        # 创建定时器，每1秒显示一次位置
        self.timer = rospy.Timer(rospy.Duration(1), self.display_position)
        
        rospy.loginfo("位置订阅器初始化完成")
        rospy.loginfo("等待接收位置数据...")

    def pose_callback(self, msg):
        """位置消息回调函数"""
        # 从pose_gt消息中获取位置信息
        self.position.x = msg.pose.pose.position.x
        self.position.y = msg.pose.pose.position.y
        self.position.z = msg.pose.pose.position.z

    def display_position(self, event):
        """显示位置信息"""
        rospy.loginfo("\n=== 航行器位置信息 ===")
        rospy.loginfo("当前位置:")
        rospy.loginfo("  X: %.3f m", self.position.x)
        rospy.loginfo("  Y: %.3f m", self.position.y)
        rospy.loginfo("  Z: %.3f m", self.position.z)
        rospy.loginfo("===================\n")

    def run(self):
        """运行节点"""
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("节点已停止")

def main():
    try:
        subscriber = PositionSubscriber()
        subscriber.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("程序正常终止")
    except Exception as e:
        rospy.logerr("程序启动失败: %s" % str(e))

if __name__ == '__main__':
    main() 