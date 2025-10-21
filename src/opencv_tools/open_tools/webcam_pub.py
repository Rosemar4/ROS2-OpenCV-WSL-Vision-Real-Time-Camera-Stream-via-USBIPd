#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class WebcamPub(Node):
    def __init__(self):
        super().__init__('webcam_pub')
        
        # Publisher
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Camera setup with MJPEG format
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        
        if not self.cap.isOpened():
        self.get_logger().error('Failed to open camera')
        return
        # Set MJPEG format for better compatibility
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
    
        self.get_logger().info('Camera opened successfully with MJPEG format')
        self.get_logger().info(f'Resolution: {int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}')
        
        # Timer for publishing frames
        self.timer = self.create_timer(0.033, self.timer_callback)  # ~30 FPS
        
    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if ret:
            # Convert to ROS Image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            
            # Publish
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn('Failed to read frame from camera')
    
    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()
