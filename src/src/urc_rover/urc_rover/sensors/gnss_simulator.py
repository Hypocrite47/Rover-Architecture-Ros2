#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from turtlesim.msg import Pose
import random

class GNSSSimulator(Node):
    def __init__(self):
        super().__init__('gnss_simulator')
        
        # Publisher for GNSS data
        self.gnss_pub = self.create_publisher(NavSatFix, '/gnss/fix', 10)
        
        # Subscribe to turtle pose
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        # Base coordinates (simulated Mars coordinates)
        self.base_lat = 38.4032  # Example: Mars-like coordinates
        self.base_lon = -110.7911
        
        self.get_logger().info('GNSS Simulator Ready!')
    
    def pose_callback(self, msg):
        # Convert turtle position to fake GPS coordinates
        # Turtle space is 0-11, we scale this to small GPS offsets
        
        lat_offset = (msg.x - 5.5) * 0.0001  # Center around 5.5
        lon_offset = (msg.y - 5.5) * 0.0001
        
        # Create GNSS message
        gnss_msg = NavSatFix()
        gnss_msg.header.stamp = self.get_clock().now().to_msg()
        gnss_msg.header.frame_id = 'gnss_frame'
        
        gnss_msg.latitude = self.base_lat + lat_offset
        gnss_msg.longitude = self.base_lon + lon_offset
        gnss_msg.altitude = 1350.0 + random.uniform(-2, 2)  # Mars-like elevation
        
        gnss_msg.status.status = 0  # Fix available
        gnss_msg.status.service = 1  # GPS
        
        self.gnss_pub.publish(gnss_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GNSSSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
