#!/usr/bin/env python3

# CameraPublisher Node
# Author: Aleksandar Haber (Modified by Trainee for SIWES implementation)
# Date: March 2024 (Modified October 2025)
# License: This code is the ownership of Aleksandar Haber

# Import the libraries
import cv2
import threading
import time

# These are ROS2 package modules and libraries
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge

# The argument "Node" means that the PublisherNode class inherits (or is a child of)
# the class called Node. The Node class is a standard ROS2 class
class CameraPublisherNode(Node):

    # constructor
    def __init__(self):
        super().__init__("publisher_node")

        # This function is used to initialize the attributes of the parent class
        
        # here we create an instance of the OpenCV videoCapture object
        # NOTE: Using index 0 and forcing the V4L2 backend for WSL stability
        self.cameraDeviceNumber = 0
        self.camera = cv2.VideoCapture(self.cameraDeviceNumber, cv2.CAP_V4L2)
        
        # CRITICAL: Set MJPEG format for better compatibility with laptop webcams
        self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
        
        # Set a stable resolution to reduce strain on the WSL attachment
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.camera.set(cv2.CAP_PROP_FPS, 30)
        
        # Reduce buffer size to prevent frame queuing issues
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # Increase timeout for V4L2
        self.camera.set(cv2.CAP_PROP_AUTOFOCUS, 0)

        # CRITICAL CHECK: Ensure the camera opened successfully
        if not self.camera.isOpened():
            self.get_logger().error('FATAL: Camera failed to open. Check USBIPd attachment and ensure index 0 is correct.')
            raise RuntimeError('Camera failed to initialize.') 

        self.get_logger().info(f'Camera opened with MJPEG format: {int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))} @ {int(self.camera.get(cv2.CAP_PROP_FPS))} FPS')

        # CvBridge is used to convert OpenCV frames to ROS2 messages that can be sent throught the topics
        self.bridge = CvBridge()
        
        # name of the topic used to transfer the camera images
        self.topicName = 'topic_camera_image'

        # the queue size for messages
        self.queueSize = 10

        # here, the function 'self.create_publisher' creates the publisher
        self.publisher = self.create_publisher(Image, self.topicName, self.queueSize)

        # communication period in seconds
        self.periodCommunication = 1.0

        # create the timer that calls the function self.timer_callback
        self.timer = self.create_timer(self.periodCommunication, self.timer_callback)

        # this is the counter tracking how many images are published
        self.i = 0
        
        # Thread-safe frame storage
        self.current_frame = None
        self.frame_lock = threading.Lock()
        
        # Start background thread to continuously read frames
        self.camera_thread_running = True
        self.camera_thread = threading.Thread(target=self._camera_read_thread, daemon=True)
        self.camera_thread.start()
        
        self.get_logger().info('Camera publisher node initialized successfully.')
    
    def _camera_read_thread(self):
        """Background thread that continuously reads frames from the camera"""
        while self.camera_thread_running:
            success, frame = self.camera.read()
            if success and frame is not None:
                with self.frame_lock:
                    self.current_frame = frame
            else:
                self.get_logger().warn('Warning: Failed to read frame in background thread')
            time.sleep(0.033)  # Read at ~30 FPS in background
        
    # this is the callback function that is called every self.periodCommunication seconds
    def timer_callback(self):
        # Get the latest frame from the background thread
        with self.frame_lock:
            frame = self.current_frame
        
        if frame is not None:
            # here, we convert the OpenCV frame to 
            # ROS2/Image message using the bridge object 'cv2_to_imgmsg(frame)
            ros2ImageMessage = self.bridge.cv2_to_imgmsg(frame)
            
            # publish the image
            self.publisher.publish(ros2ImageMessage)
            
            # Use the logger to display a message on the screen
            self.get_logger().info('Publishing image number: %d' % self.i)

            # update the image counter
            self.i += 1
        else:
            self.get_logger().warn('Warning: No frame available yet.')
    
    def destroy_node(self):
        """Clean up resources"""
        self.camera_thread_running = False
        self.camera_thread.join(timeout=2.0)
        self.camera.release()
        super().destroy_node()


# this is the main function and this is the entry point of our code
def main(args=None):
    # initialize rclpy
    rclpy.init(args=args)

    # create the publisher object
    try:
        publisherObject = CameraPublisherNode()
    except RuntimeError:
        # Exit if the camera failed to open during initialization
        rclpy.shutdown()
        return

    # here we spin, and the callback timer function is called recursively
    rclpy.spin(publisherObject)

    # destroy publisherObject
    publisherObject.destroy_node()

    # shutdown
    rclpy.shutdown()


if __name__ == '__main__':
    main()
