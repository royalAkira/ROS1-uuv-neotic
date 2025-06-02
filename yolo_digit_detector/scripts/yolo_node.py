#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import torch
import cv2
import os

class YoloV5ROS:
    def __init__(self):
        # 模型路径（使用绝对路径）
        model_path = '/home/royal/catkin_ws/src/yolo_digit_detector/models/best.pt'
        rospy.loginfo(f"Loading YOLOv5 model from {model_path}")
        try:
            self.model = torch.hub.load('/home/royal/catkin_ws/src/yolo_digit_detector/yolov5', 'custom', path=model_path, source='local')
            self.model.conf = 0.4  # 置信度阈值
            rospy.loginfo("YOLOv5 model loaded successfully")
        except Exception as e:
            rospy.logerr(f"Failed to load YOLOv5 model: {e}")
            raise
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/rexrov/rexrov/camera/camera_image', Image, self.image_callback)
        self.result_pub = rospy.Publisher('/yolov5/results', String, queue_size=10)
        self.detection_pub = rospy.Publisher('/yolov5/detection_status', Bool, queue_size=10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"CV bridge error: {e}")
            return

        results = self.model(cv_image)
        result_df = results.pandas().xyxy[0]

        # 发布识别信息
        result_str = result_df.to_json()
        self.result_pub.publish(result_str)
        rospy.loginfo(result_str)

        # 发布检测状态
        has_detection = len(result_df) > 0
        self.detection_pub.publish(Bool(has_detection))
        if has_detection:
            rospy.loginfo("已检测到目标，前往下一个地点")
        else:
            rospy.loginfo("No target detected")

        # 可视化（可选）
        results.render()
        img = results.render()[0]
        cv2.imshow("YOLOv5 Detection", img)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('yolov5_ros_node')
    YoloV5ROS()
    rospy.spin()
