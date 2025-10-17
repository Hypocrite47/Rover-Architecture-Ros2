#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class LEDIndicator(Node):
    def __init__(self):
        super().__init__('led_indicator')
        
        # Subscribe to LED commands
        self.led_sub = self.create_subscription(
            String,
            '/led_status',
            self.led_callback,
            10
        )
        
        self.current_status = 'MANUAL'
        self.colors = {
            'AUTONOMOUS': '\033[91m',  # Red
            'MANUAL': '\033[94m',      # Blue
            'TARGET_REACHED': '\033[92m',  # Green
            'RESET': '\033[0m'         # Reset color
        }
        
        self.get_logger().info('LED Indicator Ready!')
        self.display_status('MANUAL')
    
    def led_callback(self, msg):
        status = msg.data.upper()
        self.display_status(status)
    
    def display_status(self, status):
        self.current_status = status
        color = self.colors.get(status, self.colors['RESET'])
        
        # Print colored status
        print(f"\n{'='*50}")
        print(f"{color}LED STATUS: {status}{self.colors['RESET']}")
        print(f"{'='*50}\n")
        
        self.get_logger().info(f'LED: {status}')

def main(args=None):
    rclpy.init(args=args)
    node = LEDIndicator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
