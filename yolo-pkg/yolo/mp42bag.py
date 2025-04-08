#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class VideoPublisher(Node):
    def __init__(self, video_file, topic_name="/camera/image_raw", fps=30):
        super().__init__('video_publisher')

        self.publisher = self.create_publisher(Image, topic_name, 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(video_file)
        self.timer_period = 1.0 / fps  # 計算時間間隔
        self.timer = self.create_timer(self.timer_period, self.publish_frame)

        self.get_logger().info(f"開始播放 {video_file} -> Topic: {topic_name}")

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("影片播放結束")
            self.cap.release()
            rclpy.shutdown()
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        self.publisher.publish(msg)
        self.get_logger().info(f"發佈影像 {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")

def main(args=None):
    rclpy.init(args=args)
    video_file = "src/yolo_ros/person480.mp4" 
    topic_name = "/camera/image_raw"
    fps = 29.97

    node = VideoPublisher(video_file, topic_name, fps)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
