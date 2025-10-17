#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from urc_rover_interfaces.action import NavigateToGoal
import time

class ServicingMission(Node):
    def __init__(self):
        super().__init__('servicing_mission')
        
        # Action client for navigation
        self._nav_client = ActionClient(self, NavigateToGoal, 'navigate_to_goal')
        
        # Publishers
        self.led_pub = self.create_publisher(String, '/led_status', 10)
        
        # Equipment servicing locations
        self.equipment = [
            {
                'x': 3.0, 'y': 3.0,
                'name': 'Solar_Panel_Array',
                'tasks': ['Check connections', 'Clean surface', 'Test voltage']
            },
            {
                'x': 8.0, 'y': 8.0,
                'name': 'Communication_Antenna',
                'tasks': ['Adjust alignment', 'Check signal', 'Secure mounting']
            },
            {
                'x': 5.5, 'y': 5.5,
                'name': 'Weather_Station',
                'tasks': ['Replace sensors', 'Calibrate', 'Update firmware']
            }
        ]
        
        self.get_logger().info('Equipment Servicing Mission Ready!')
    
    def run_mission(self):
        """Execute equipment servicing mission"""
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('Starting Equipment Servicing Mission')
        self.get_logger().info('='*50 + '\n')
        
        self.led_pub.publish(String(data='AUTONOMOUS'))
        
        for i, equipment in enumerate(self.equipment):
            self.get_logger().info(f"\n--- Equipment {i+1}/{len(self.equipment)}: {equipment['name']} ---")
            
            # Navigate to equipment
            self.get_logger().info(f"Navigating to {equipment['name']}...")
            if not self.navigate_to(equipment['x'], equipment['y']):
                self.get_logger().error(f"Failed to reach {equipment['name']}")
                continue
            
            # Arrived
            self.led_pub.publish(String(data='TARGET_REACHED'))
            self.get_logger().info(f"Arrived at {equipment['name']}!")
            time.sleep(1)
            
            # Perform servicing tasks
            self.get_logger().info(f"Performing maintenance on {equipment['name']}:")
            for j, task in enumerate(equipment['tasks']):
                self.get_logger().info(f"  [{j+1}/{len(equipment['tasks'])}] {task}...")
                time.sleep(2)  # Simulate task execution
                self.get_logger().info(f"  ✓ {task} complete")
            
            self.get_logger().info(f"{equipment['name']} servicing complete!\n")
            
            # Back to autonomous mode
            if i < len(self.equipment) - 1:
                self.led_pub.publish(String(data='AUTONOMOUS'))
                time.sleep(1)
        
        self.led_pub.publish(String(data='MANUAL'))
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('Equipment Servicing Mission Complete!')
        self.get_logger().info('='*50 + '\n')
    
    def navigate_to(self, x, y):
        """Navigate to equipment location"""
        self._nav_client.wait_for_server()
        
        goal = NavigateToGoal.Goal()
        goal.target_x = x
        goal.target_y = y
        goal.waypoint_type = 'gnss'
        
        future = self._nav_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        return result.success
    
    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Distance remaining: {feedback.distance_remaining:.2f}',
            throttle_duration_sec=1.0
        )

def main(args=None):
    rclpy.init(args=args)
    node = ServicingMission()
    
    try:
        # Wait for other nodes
        time.sleep(2)
        node.run_mission()
    except KeyboardInterrupt:
        node.get_logger().info('Servicing mission interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
