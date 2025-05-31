#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

class FloatController:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('float_controller', anonymous=True)
        
        # 创建发布者
        self.cmd_vel_pub = rospy.Publisher('/rexrov/cmd_vel', Twist, queue_size=10)
        
        # 保存终端设置
        self.settings = termios.tcgetattr(sys.stdin)
        
        rospy.loginfo("上浮控制器初始化完成")
        rospy.loginfo("按 'K' 开始上浮，按 'Q' 停止...")

    def get_key(self):
        """获取键盘输入"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        """运行控制器"""
        rate = rospy.Rate(10)  # 10Hz控制频率
        is_running = False
        
        # 等待按K开始
        while not rospy.is_shutdown():
            key = self.get_key()
            if key == 'k' or key == 'K':
                rospy.loginfo("开始上浮...")
                is_running = True
                break
            elif key == 'q' or key == 'Q':
                rospy.loginfo("程序退出")
                return
            rate.sleep()
        
        while not rospy.is_shutdown() and is_running:
            # 检查是否有键盘输入（非阻塞方式）
            try:
                key = self.get_key()
                if key == 'q' or key == 'Q':
                    rospy.loginfo("收到停止命令，正在停止航行器...")
                    # 发送零速度命令
                    cmd = Twist()
                    self.cmd_vel_pub.publish(cmd)
                    is_running = False
                    continue
            except:
                pass

            # 创建上浮命令
            cmd = Twist()
            cmd.linear.z = 0.3  # 设置上浮速度为0.3 m/s
            self.cmd_vel_pub.publish(cmd)
            
            rate.sleep()
        
        # 确保发送最后的零速度命令
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        rospy.loginfo("航行器已停止")

def main():
    try:
        controller = FloatController()
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