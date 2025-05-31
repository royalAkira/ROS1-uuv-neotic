#!/usr/bin/env python
# coding: utf-8

import rospy
import select
import sys
import termios
import tty
from geometry_msgs.msg import Twist

class RexrovController:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('rexrov_cmd_vel_controller', anonymous=True)
        
        # 创建发布者
        self.pub = rospy.Publisher('/rexrov/cmd_vel', Twist, queue_size=10)
        
        # 创建Twist消息
        self.cmd = Twist()
        self._setup_twist_message_init()  # 初始化为静止状态
        
        # 保存终端设置
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        rospy.loginfo("Rexrov控制器初始化完成:")
        rospy.loginfo("按 'S' 键开始运动，按 'Q' 键停止运动")
        rospy.loginfo("按 'Ctrl+C' 关闭控制器")

    def _setup_twist_message(self):
        """设置Twist消息的运动值"""
        self.cmd.linear.x = 0.0
        self.cmd.linear.y = 0.0
        self.cmd.linear.z = -10.0
        self.cmd.angular.x = 0.0
        self.cmd.angular.y = 0.0
        self.cmd.angular.z = 0.0

    def _setup_twist_message_init(self):
        """设置Twist消息的初始值（静止状态）"""
        self.cmd.linear.x = 0.0
        self.cmd.linear.y = 0.0
        self.cmd.linear.z = 0.0
        self.cmd.angular.x = 0.0
        self.cmd.angular.y = 0.0
        self.cmd.angular.z = 0.0

    def _start_movement(self):
        """开始运动"""
        self._setup_twist_message()
        rospy.loginfo("开始运动")

    def _stop_movement(self):
        """停止运动"""
        self._setup_twist_message_init()
        rospy.loginfo("停止运动")

    def _get_key(self):
        """获取键盘输入（非阻塞）"""
        try:
            # 设置终端为raw模式
            tty.setraw(sys.stdin.fileno())
            # 等待输入，但设置超时
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                return key
        except:
            pass
        finally:
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        return None

    def cleanup(self):
        """清理资源"""
        try:
            rospy.loginfo("正在清理资源...")
            self._stop_movement()  # 确保机器人停止
            # 发布几次停止命令以确保收到
            for _ in range(3):
                self.pub.publish(self.cmd)
                rospy.sleep(0.1)
        finally:
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            rospy.loginfo("资源清理完成")

    def run(self):
        """运行控制器主循环"""
        rate = rospy.Rate(10)
        rospy.loginfo("等待键盘输入...")
        
        try:
            while not rospy.is_shutdown():
                # 检查键盘输入
                key = self._get_key()
                if key == 's' or key == 'S':
                    self._start_movement()
                elif key == 'q' or key == 'Q':
                    self._stop_movement()
                
                self.pub.publish(self.cmd)
                rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("收到Ctrl+C信号，正在关闭控制器...")
        except Exception as e:
            rospy.logerr("运行过程中发生错误: %s" % str(e))
        finally:
            self.cleanup()  # 确保清理资源

def main():
    try:
        controller = RexrovController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("程序正常终止")
    except Exception as e:
        rospy.logerr("程序启动失败: %s" % str(e))

if __name__ == '__main__':
    main() 