#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from urc_rover_interfaces.action import NavigateToGoal
import time

class ScienceMission(Node):
    def __init__(self):
        super().__init__('science_mission')
        
        # Action client for navigation
        self._nav_client = ActionClient(self, NavigateToGoal, 'navigate_to_goal')
        
        # Publishers
        self.led_pub = self.create_publisher(String, '/led_status', 10)
        self.capture_pub = self.create_publisher(String, '/camera/capture_request', 10)
        
        # Subscribers
        self.gnss_sub = self.create_subscription(NavSatFix, '/gnss/fix', self.gnss_callback, 10)
        self.confirm_sub = self.create_subscription(
            String,
            '/camera/capture_confirm',
            self.confirm_callback,
            10
        )
        
        self.current_gnss = None
        self.capture_confirmed = False
        
        # Science sites (x, y coordinates in turtlesim - keep away from walls)
        self.sites = [
            {'x': 2.5, 'y': 2.5, 'name': 'Site_Alpha'},
            {'x': 8.5, 'y': 2.5, 'name': 'Site_Beta'},
            {'x': 8.5, 'y': 8.5, 'name': 'Site_Gamma'},
        ]
        
        self.current_site = 0
        
        self.get_logger().info('Science Mission Ready!')
    
    def gnss_callback(self, msg):
        self.current_gnss = msg
    
    def confirm_callback(self, msg):
        self.capture_confirmed = True
        self.get_logger().info(f'Capture confirmed: {msg.data}')
    
    def run_mission(self):
        self.get_logger().info('Starting Science Mission...')
        self.led_pub.publish(String(data='AUTONOMOUS'))
        
        for site in self.sites:
            self.get_logger().info(f"\n--- Visiting {site['name']} ---")
            
            # Navigate to site
            if not self.navigate_to(site['x'], site['y'], 'gnss'):
                self.get_logger().error(f"Failed to reach {site['name']}")
                continue
            
            # Arrived
            self.led_pub.publish(String(data='TARGET_REACHED'))
            self.get_logger().info(f"Arrived at {site['name']}!")
            
            # Wait a moment
            time.sleep(2)
            
            # Capture image with GNSS data
            self.capture_site_data(site['name'])
            
            # Wait for confirmation
            time.sleep(2)
            
            self.get_logger().info(f"{site['name']} documentation complete!\n")
        
        self.led_pub.publish(String(data='MANUAL'))
        self.get_logger().info('Science Mission Complete!')
    
    def navigate_to(self, x, y, waypoint_type):
        self.get_logger().info(f'Navigating to ({x}, {y})...')
        
        # Wait for action server with timeout
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
        
        # Wait for goal to be accepted
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
        
        goal_handle = send_goal_future.result()
        if not goal_handle:
            self.get_logger().error('Goal was rejected by server')
            return False
            
        if not goal_handle.accepted:
            self.get_logger().error('Goal not accepted')
            return False
        
        self.get_logger().info('Goal accepted, waiting for result...')
        
        # Wait for result with timeout
        result_future = goal_handle.get_result_async()
        
        # Spin with timeout
        timeout = 60.0  # 60 seconds max
        start_time = time.time()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().error('Navigation timeout!')
                goal_handle.cancel_goal_async()
                return False
        
        result = result_future.result().result
        
        if result.success:
            self.get_logger().info(f'Navigation successful!')
        else:
            self.get_logger().error(f'Navigation failed: {result.message}')
        
        return result.success
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Position: ({feedback.current_x:.2f}, {feedback.current_y:.2f}), '
            f'Distance: {feedback.distance_remaining:.2f}',
            throttle_duration_sec=2.0
        )
    
    def capture_site_data(self, site_name):
        self.get_logger().info('Capturing site image and GNSS...')
        
        if self.current_gnss:
            self.get_logger().info(
                f'GNSS: Lat={self.current_gnss.latitude:.6f}, '
                f'Lon={self.current_gnss.longitude:.6f}'
            )
        
        # Request image capture
        self.capture_confirmed = False
        msg = String()
        msg.data = f'{0.0},{0.0},science_{site_name}'
        self.capture_pub.publish(msg)
        
        # Wait for confirmation
        timeout = 5.0
        start = time.time()
        while not self.capture_confirmed and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    node = ScienceMission()
    
    try:
        # Wait a bit for other nodes to start
        time.sleep(2)
        node.run_mission()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
