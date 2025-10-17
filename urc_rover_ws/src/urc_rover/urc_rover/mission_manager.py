#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        
        # Declare mission mode parameter
        descriptor = ParameterDescriptor(description='Mission mode: science, delivery, autonomous, servicing')
        self.declare_parameter('mission_mode', 'science', descriptor)
        
        # Add parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.current_mode = self.get_parameter('mission_mode').value
        
        self.get_logger().info(f'Mission Manager Started')
        self.get_logger().info(f'Current mission mode: {self.current_mode}')
        self.get_logger().info('Change mode with: ros2 param set /mission_manager mission_mode <mode>')
    
    def parameter_callback(self, params):
        from rcl_interfaces.msg import SetParametersResult
        
        for param in params:
            if param.name == 'mission_mode':
                new_mode = param.value
                self.get_logger().info(f'Mission mode changing: {self.current_mode} -> {new_mode}')
                self.current_mode = new_mode
                self.get_logger().info(f'Mission mode is now: {new_mode}')
        
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
