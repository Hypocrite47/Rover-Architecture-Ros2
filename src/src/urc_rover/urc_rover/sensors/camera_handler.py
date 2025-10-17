#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
import json

class CameraHandler(Node):
    def __init__(self):
        super().__init__('camera_handler')
        
        self.bridge = CvBridge()
        
        # Try to open webcam
        self.cap = cv2.VideoCapture(0)
        
        if not self.cap.isOpened():
            self.get_logger().warn('No webcam found, using dummy images')
            self.use_dummy = True
        else:
            self.use_dummy = False
            self.get_logger().info('Webcam opened successfully!')
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.confirm_pub = self.create_publisher(String, '/camera/capture_confirm', 10)
        
        # Subscriber for capture requests
        self.capture_sub = self.create_subscription(
            String,
            '/camera/capture_request',
            self.capture_callback,
            10
        )
        
        # Timer to publish images
        self.timer = self.create_timer(0.1, self.publish_image)
        
        # Save directory
        self.save_dir = os.path.expanduser('~/urc_rover_ws/src/urc_rover/data/images')
        os.makedirs(self.save_dir, exist_ok=True)
        
        self.get_logger().info('Camera Handler Ready!')
    
    def publish_image(self):
        if self.use_dummy:
            img = self.create_dummy_image()
        else:
            ret, img = self.cap.read()
            if not ret:
                img = self.create_dummy_image()
        
        try:
            ros_img = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            ros_img.header.stamp = self.get_clock().now().to_msg()
            self.image_pub.publish(ros_img)
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
    
    def create_dummy_image(self):
        import numpy as np
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img[:, :] = (100, 150, 200)
        
        text = datetime.now().strftime('%H:%M:%S')
        cv2.putText(img, f'DUMMY CAM: {text}', (50, 240), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        return img
    
    def capture_callback(self, msg):
        self.get_logger().info('Capture request received!')
        
        # Parse request (format: "x,y,mission_type")
        try:
            data = msg.data.split(',')
            x, y = float(data[0]), float(data[1])
            mission = data[2] if len(data) > 2 else 'unknown'
        except:
            x, y, mission = 0.0, 0.0, 'unknown'
        
        # Capture image
        if self.use_dummy:
            img = self.create_dummy_image()
        else:
            ret, img = self.cap.read()
            if not ret:
                img = self.create_dummy_image()
        
        # Save image
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'{mission}_{timestamp}.jpg'
        filepath = os.path.join(self.save_dir, filename)
        cv2.imwrite(filepath, img)
        
        # Save metadata
        metadata = {
            'timestamp': timestamp,
            'position': {'x': x, 'y': y},
            'mission': mission,
            'image_file': filename
        }
        
        meta_file = filepath.replace('.jpg', '_metadata.json')
        with open(meta_file, 'w') as f:
            json.dump(metadata, f, indent=2)
        
        self.get_logger().info(f'Saved: {filename}')
        
        # Confirm
        confirm = String()
        confirm.data = f'captured:{filename}'
        self.confirm_pub.publish(confirm)
    
    def __del__(self):
        if not self.use_dummy:
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = CameraHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
