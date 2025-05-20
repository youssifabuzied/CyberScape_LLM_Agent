#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import time
import math
import torch
from ultralytics import YOLO

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('yolov8_object_detector')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.detection_pub = self.create_publisher(Image, 'yolo/image_detected', 10)
        
        # Subscribers
        self.camera_sub = self.create_subscription(
            Image, 
            'camera/image_raw',
            self.camera_callback, 
            10)
        
        # Variables
        self.bridge = CvBridge()
        self.target_object = ""
        self.object_detected = False
        self.object_coordinates = None
        self.scanning_complete = False
        self.current_frame = None
        
        # Load YOLOv8 model
        self.model = YOLO('yolov8n.pt')  # Use smaller model for faster inference
        
        self.get_logger().info('YOLOv8 object detector initialized')
    
    def set_target_object(self, object_name):
        """Set the target object to detect"""
        self.target_object = object_name.lower()
        self.object_detected = False
        self.object_coordinates = None
        self.scanning_complete = False
        self.get_logger().info(f'Target object set to: {self.target_object}')
        
    def camera_callback(self, msg):
        """Process camera images and run YOLOv8 detection"""
        if self.target_object == "":
            return
            
        # Convert ROS Image message to OpenCV image
        self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Run YOLOv8 detection
        results = self.model(self.current_frame)
        
        # Process results
        for result in results:
            boxes = result.boxes
            for box in boxes:
                cls = int(box.cls[0])
                cls_name = self.model.names[cls].lower()
                
                if cls_name == self.target_object:
                    self.object_detected = True
                    
                    # Get coordinates (normalized)
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    
                    # Calculate center
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    confidence = float(box.conf[0])
                    
                    self.object_coordinates = (center_x, center_y, confidence)
                    self.get_logger().info(f'Detected {self.target_object} at coordinates: {self.object_coordinates}')
                    
                    # Publish annotated image
                    annotated_frame = results[0].plot()
                    self.detection_pub.publish(self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8'))
                    return
    
    def rotate_and_scan(self):
        """Rotate the robot 360 degrees while looking for the target object"""
        self.get_logger().info(f'Starting 360-degree scan for {self.target_object}')
        
        # Create Twist message for rotation
        twist = Twist()
        twist.angular.z = 0.3  # Adjust rotation speed as needed
        
        scan_start_time = time.time()
        total_angle = 0.0
        last_time = scan_start_time
        
        # Start rotation
        while not self.scanning_complete and not self.object_detected:
            # Publish rotation command
            self.cmd_vel_pub.publish(twist)
            
            # Update current angle (approximate)
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            total_angle += twist.angular.z * dt
            
            # Check if we've completed a full rotation
            if total_angle >= 2 * math.pi:
                self.scanning_complete = True
                
            # Process callbacks
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Stop rotation
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        if self.object_detected:
            self.get_logger().info(f'Found {self.target_object} at {self.object_coordinates}')
            return True, self.object_coordinates
        else:
            self.get_logger().info(f'{self.target_object} not found after complete scan')
            return False, None

def detect_object(object_name):
    """Function to be called by user to detect an object"""
    rclpy.init()
    detector = ObjectDetector()
    
    # Set the target object
    detector.set_target_object(object_name)
    
    # Perform the scan
    result, coordinates = detector.rotate_and_scan()
    
    # Clean up
    detector.destroy_node()
    rclpy.shutdown()
    
    return result, coordinates

if __name__ == '__main__':
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python3 yolov8_detector.py <object_name>")
        sys.exit(1)
    
    object_name = sys.argv[1]
    result, coords = detect_object(object_name)
    
    if result:
        print(f"Object '{object_name}' detected at coordinates: {coords}")
    else:
        print(f"Object '{object_name}' not found")
