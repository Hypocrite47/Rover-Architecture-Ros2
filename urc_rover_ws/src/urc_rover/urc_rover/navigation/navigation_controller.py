#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from urc_rover_interfaces.action import NavigateToGoal
import math
import time

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Use reentrant callback group to allow parallel callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(
            Pose, 
            '/turtle1/pose', 
            self.pose_callback, 
            10,
            callback_group=self.callback_group
        )
        
        self.current_pose = None
        
        self._action_server = ActionServer(
            self,
            NavigateToGoal,
            'navigate_to_goal',
            self.execute_navigation,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Navigation Controller Ready!')
    
    def pose_callback(self, msg):
        self.current_pose = msg
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def execute_navigation(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info(f'Navigation goal received: ({goal.target_x}, {goal.target_y})')
        
        # Wait for pose
        wait_count = 0
        while self.current_pose is None and wait_count < 50:
            rclpy.spin_once(self, timeout_sec=0.1)
            wait_count += 1
        
        if self.current_pose is None:
            self.get_logger().error('No pose received!')
            goal_handle.abort()
            return NavigateToGoal.Result(success=False, message='No pose')
        
        self.get_logger().info(f'Starting from ({self.current_pose.x:.2f}, {self.current_pose.y:.2f})')
        
        feedback = NavigateToGoal.Feedback()
        result = NavigateToGoal.Result()
        
        # Control parameters
        linear_gain = 1.5
        angular_gain = 4.0
        max_linear_speed = 2.0
        max_angular_speed = 2.0
        distance_tolerance = 0.3
        angle_tolerance = 0.15  # ~8.5 degrees
        
        iteration = 0
        max_iterations = 600  # 60 seconds at 10Hz
        
        while iteration < max_iterations:
            # Spin to update pose
            rclpy.spin_once(self, timeout_sec=0.01)
            
            if self.current_pose is None:
                iteration += 1
                time.sleep(0.1)
                continue
            
            # Calculate distance and angle to goal
            dx = goal.target_x - self.current_pose.x
            dy = goal.target_y - self.current_pose.y
            distance = math.sqrt(dx**2 + dy**2)
            
            # Update feedback
            feedback.current_x = self.current_pose.x
            feedback.current_y = self.current_pose.y
            feedback.distance_remaining = distance
            goal_handle.publish_feedback(feedback)
            
            # Check if reached goal
            if distance < distance_tolerance:
                self.stop()
                self.get_logger().info(f'✓ Goal reached at ({self.current_pose.x:.2f}, {self.current_pose.y:.2f})')
                
                # Stop completely and wait to settle
                for _ in range(5):
                    self.stop()
                    time.sleep(0.1)
                
                goal_handle.succeed()
                result.success = True
                result.final_x = self.current_pose.x
                result.final_y = self.current_pose.y
                result.message = 'Navigation successful'
                return result
            
            # Calculate target angle and angle difference
            target_angle = math.atan2(dy, dx)
            angle_diff = self.normalize_angle(target_angle - self.current_pose.theta)
            
            cmd = Twist()
            
            # If angle difference is large, rotate in place
            if abs(angle_diff) > angle_tolerance:
                # Rotate only
                cmd.linear.x = 0.0
                cmd.angular.z = angular_gain * angle_diff
                cmd.angular.z = max(-max_angular_speed, min(max_angular_speed, cmd.angular.z))
                
                if iteration % 20 == 0:
                    self.get_logger().info(
                        f'Rotating: angle_diff={math.degrees(angle_diff):.1f}°, '
                        f'current={math.degrees(self.current_pose.theta):.1f}°, '
                        f'target={math.degrees(target_angle):.1f}°'
                    )
            else:
                # Move forward with heading correction
                cmd.linear.x = min(max_linear_speed, linear_gain * distance)
                cmd.angular.z = angular_gain * angle_diff
                cmd.angular.z = max(-max_angular_speed, min(max_angular_speed, cmd.angular.z))
                
                if iteration % 20 == 0:
                    self.get_logger().info(
                        f'Moving: dist={distance:.2f}, '
                        f'pos=({self.current_pose.x:.2f}, {self.current_pose.y:.2f}), '
                        f'speed={cmd.linear.x:.2f}'
                    )
            
            self.cmd_pub.publish(cmd)
            
            iteration += 1
            time.sleep(0.1)  # 10Hz control loop
        
        # Timeout
        self.stop()
        self.get_logger().error('Navigation timeout!')
        goal_handle.abort()
        result.success = False
        result.final_x = self.current_pose.x if self.current_pose else 0.0
        result.final_y = self.current_pose.y if self.current_pose else 0.0
        result.message = 'Navigation timeout'
        return result
    
    def stop(self):
        """Stop the turtle"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationController()
    
    # Use MultiThreadedExecutor to allow callbacks during action execution
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()