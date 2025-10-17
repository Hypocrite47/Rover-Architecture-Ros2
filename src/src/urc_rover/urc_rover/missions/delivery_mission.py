#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from std_msgs.msg import String
from urc_rover_interfaces.action import NavigateToGoal, DeliveryTask
import time

class DeliveryMission(Node):
    def __init__(self):
        super().__init__('delivery_mission')
        
        # Action client for navigation
        self._nav_client = ActionClient(self, NavigateToGoal, 'navigate_to_goal')
        
        # Action server for delivery
        self._delivery_server = ActionServer(
            self,
            DeliveryTask,
            'delivery_task',
            self.execute_delivery
        )
        
        # Publishers
        self.led_pub = self.create_publisher(String, '/led_status', 10)
        
        # Delivery items (keep coordinates away from walls)
        self.deliveries = [
            {
                'pickup_x': 2.5, 'pickup_y': 2.5,
                'delivery_x': 8.5, 'delivery_y': 8.5,
                'object_id': 'Package_A'
            },
            {
                'pickup_x': 8.5, 'pickup_y': 2.5,
                'delivery_x': 2.5, 'delivery_y': 8.5,
                'object_id': 'Package_B'
            }
        ]
        
        self.get_logger().info('Delivery Mission Ready!')
    
    def execute_delivery(self, goal_handle):
        """Execute a single delivery task"""
        self.get_logger().info('Executing delivery task...')
        goal = goal_handle.request
        feedback = DeliveryTask.Feedback()
        
        # Step 1: Navigate to pickup
        self.get_logger().info(f'Going to pickup {goal.object_id}...')
        feedback.status = 'navigating_to_pickup'
        feedback.progress = 0.0
        goal_handle.publish_feedback(feedback)
        
        if not self.navigate_to(goal.pickup_x, goal.pickup_y):
            goal_handle.abort()
            result = DeliveryTask.Result()
            result.success = False
            result.message = 'Failed to reach pickup location'
            return result
        
        # Step 2: Pick up object
        self.get_logger().info(f'Picking up {goal.object_id}...')
        feedback.status = 'picking'
        feedback.progress = 0.25
        goal_handle.publish_feedback(feedback)
        time.sleep(2)  # Simulate pickup time
        self.get_logger().info(f'{goal.object_id} picked up!')
        
        # Step 3: Navigate to delivery location
        self.get_logger().info(f'Transporting {goal.object_id} to delivery point...')
        feedback.status = 'navigating_to_delivery'
        feedback.progress = 0.5
        goal_handle.publish_feedback(feedback)
        
        if not self.navigate_to(goal.delivery_x, goal.delivery_y):
            goal_handle.abort()
            result = DeliveryTask.Result()
            result.success = False
            result.message = 'Failed to reach delivery location'
            return result
        
        # Step 4: Deliver object
        self.get_logger().info(f'Delivering {goal.object_id}...')
        feedback.status = 'delivering'
        feedback.progress = 0.75
        goal_handle.publish_feedback(feedback)
        time.sleep(2)  # Simulate delivery time
        
        # Step 5: Complete
        self.get_logger().info(f'{goal.object_id} delivered successfully!')
        feedback.status = 'complete'
        feedback.progress = 1.0
        goal_handle.publish_feedback(feedback)
        
        goal_handle.succeed()
        result = DeliveryTask.Result()
        result.success = True
        result.message = f'Successfully delivered {goal.object_id}'
        return result
    
    def navigate_to(self, x, y):
        """Navigate to a specific position"""
        self._nav_client.wait_for_server()
        
        goal = NavigateToGoal.Goal()
        goal.target_x = x
        goal.target_y = y
        goal.waypoint_type = 'gnss'
        
        future = self._nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        return result.success
    
    def run_mission(self):
        """Run complete delivery mission"""
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('Starting Delivery Mission')
        self.get_logger().info('='*50 + '\n')
        
        self.led_pub.publish(String(data='AUTONOMOUS'))
        
        for i, delivery in enumerate(self.deliveries):
            self.get_logger().info(f"\n--- Delivery {i+1}/{len(self.deliveries)}: {delivery['object_id']} ---")
            
            # Create delivery goal
            goal = DeliveryTask.Goal()
            goal.pickup_x = delivery['pickup_x']
            goal.pickup_y = delivery['pickup_y']
            goal.delivery_x = delivery['delivery_x']
            goal.delivery_y = delivery['delivery_y']
            goal.object_id = delivery['object_id']
            
            # Execute delivery manually (simulating action client)
            # Step 1: Go to pickup
            self.get_logger().info(f'Going to pickup location ({goal.pickup_x}, {goal.pickup_y})')
            if not self.navigate_to(goal.pickup_x, goal.pickup_y):
                self.get_logger().error(f'Failed to reach pickup for {goal.object_id}')
                continue
            
            # Step 2: Pick up
            self.get_logger().info(f'Picking up {goal.object_id}...')
            time.sleep(2)
            self.get_logger().info(f'✓ {goal.object_id} picked up!')
            
            # Step 3: Go to delivery
            self.get_logger().info(f'Going to delivery location ({goal.delivery_x}, {goal.delivery_y})')
            if not self.navigate_to(goal.delivery_x, goal.delivery_y):
                self.get_logger().error(f'Failed to reach delivery for {goal.object_id}')
                continue
            
            # Step 4: Deliver
            self.get_logger().info(f'Delivering {goal.object_id}...')
            time.sleep(2)
            self.get_logger().info(f'✓ {goal.object_id} delivered successfully!\n')
            
            # Brief pause between deliveries
            if i < len(self.deliveries) - 1:
                time.sleep(1)
        
        self.led_pub.publish(String(data='MANUAL'))
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('Delivery Mission Complete!')
        self.get_logger().info('='*50 + '\n')

def main(args=None):
    rclpy.init(args=args)
    node = DeliveryMission()
    
    try:
        # Wait for other nodes to start
        time.sleep(2)
        
        # Run the delivery mission
        node.run_mission()
        
        # Keep spinning to handle action server requests
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Delivery mission interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
