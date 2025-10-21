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
        
        if not self.
