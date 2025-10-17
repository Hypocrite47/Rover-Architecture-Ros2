#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from urc_rover_interfaces.action import NavigateToGoal
import time

class AutonomousNav(Node):
    def __init__(self):
        super().__init__('autonomous_nav')
        
        # Action client for navigation
        self._nav_client = ActionClient(self, NavigateToGoal, 'navigate_to_goal')
        
        # Publishers
        self.led_pub = self.create_publisher(String, '/led_status', 10)
        
        # Waypoints: 2 GNSS, 2 Vision (keep away from walls)
        self.waypoints = [
            {'x': 2.5, 'y': 2.5, 'type': 'gnss', 'name': 'GNSS_Point_1'},
            {'x': 8.5, 'y': 2.5, 'type': 'vision', 'name': 'Vision_Target_1'},
            {'x': 8.5, 'y': 8.5, 'type': 'gnss', 'name': 'GNSS_Point_2'},
            {'x': 2.5, 'y': 8.5, 'type': 'vision', 'name': 'Vision_Target_2'},
        ]
        
        self.get_logger().info('Autonomous Navigation Ready!')
    
    def run_mission(self):
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('Starting Autonomous Navigation Mission')
        self.get_logger().info('='*50 + '\n')
        
        # Set LED to RED (Autonomous mode)
        self.led_pub.publish(String(data='AUTONOMOUS'))
        
        for i, waypoint in enumerate(self.waypoints):
            self.get_logger().info(f"\n--- Waypoint {i+1}/4: {waypoint['name']} ---")
            self.get_logger().info(f"Type: {waypoint['type'].upper()}")
            self.get_logger().info(f"Target: ({waypoint['x']}, {waypoint['y']})")
            
            # Navigate to waypoint
            if not self.navigate_to(waypoint['x'], waypoint['y'], waypoint['type']):
                self.get_logger().error(f"Failed to reach {waypoint['name']}")
                continue
            
            # Reached target - Set LED to GREEN
            self.led_pub.publish(String(data='TARGET_REACHED'))
            self.get_logger().info(f"✓ Reached {waypoint['name']}!")
            
            # Simulate target verification
            if waypoint['type'] == 'vision':
                self.get_logger().info('Vision system detected target marker')
            else:
                self.get_logger().info('GNSS coordinates confirmed')
            
            # Wait before next waypoint
            self.get_logger().info('Waiting 3 seconds before next waypoint...')
            time.sleep(3)
            
            # Back to autonomous (RED)
            if i < len(self.waypoints) - 1:
                self.led_pub.publish(String(data='AUTONOMOUS'))
                self.get_logger().info('Proceeding to next waypoint...\n')
        
        # Mission complete - Set LED to BLUE (Manual mode)
        self.led_pub.publish(String(data='MANUAL'))
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('Autonomous Navigation Mission Complete!')
        self.get_logger().info('='*50 + '\n')
    
    def navigate_to(self, x, y, waypoint_type):
        self.get_logger().info(f'Sending navigation goal to ({x}, {y})...')
        
        # Wait for action server
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available!')
            return False
        
        # Create goal
        goal = NavigateToGoal.Goal()
        goal.target_x = x
        goal.target_y = y
        goal.waypoint_type = waypoint_type
        
        # Send goal
        send_goal_future = self._nav_client.send_goal_async(
            goal, 
            feedback_callback=self.feedback_callback
        )
        
        # Wait for acceptance
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
        
        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Goal rejected by navigation server')
            return False
        
        self.get_logger().info('Goal accepted, navigating...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        
        # Spin until complete with timeout
        start_time = time.time()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > 70.0:  # 70 second timeout
                self.get_logger().error('Navigation result timeout!')
                return False
        
        result = result_future.result().result
        
        if result.success:
            self.get_logger().info(f'Navigation successful to ({result.final_x:.2f}, {result.final_y:.2f})')
        else:
            self.get_logger().error(f'Navigation failed: {result.message}')
        
        return result.success
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Position: ({feedback.current_x:.2f}, {feedback.current_y:.2f}), '
            f'Distance: {feedback.distance_remaining:.2f}m',
            throttle_duration_sec=2.0
        )

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNav()
    
    try:
        # Wait for other nodes
        time.sleep(2)
        node.run_mission()
        
        # Keep node alive to handle any remaining callbacks
        time.sleep(1)
    except KeyboardInterrupt:
        node.get_logger().info('Mission interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()