#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import time

class GazeboSimTester(Node):
    def __init__(self):
        super().__init__('gazebo_sim_tester')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.left_steer_pub = self.create_publisher(Float64, '/front_left_steer_joint/cmd_position', 10)
        self.right_steer_pub = self.create_publisher(Float64, '/front_right_steer_joint/cmd_position', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/ackermann_robot/odometry',
            self.odom_callback,
            10)
        
        self.initial_pose = None
        self.current_pose = None
        self.test_completed = False
        self.get_logger().info('Gazebo Sim Tester node initialized')
        
    def odom_callback(self, msg):
        if self.initial_pose is None:
            self.initial_pose = msg.pose.pose
            self.get_logger().info('Initial pose recorded')
        
        self.current_pose = msg.pose.pose
    
    def run_test(self):
        self.get_logger().info('Starting Gazebo simulation test')
        
        # Wait for odometry to be received
        timeout = 10.0  # seconds
        start_time = time.time()
        
        while self.initial_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().error('Timed out waiting for odometry data')
                return False
        
        self.get_logger().info('Connected to simulation, sending test commands')
        
        # Test 1: Send steering commands
        self.get_logger().info('Test 1: Sending steering commands')
        steer_msg = Float64()
        steer_msg.data = 0.3  # ~17 degrees steering angle
        
        self.left_steer_pub.publish(steer_msg)
        self.right_steer_pub.publish(steer_msg)
        time.sleep(2.0)
        
        # Test 2: Send forward velocity command
        self.get_logger().info('Test 2: Sending forward velocity command')
        vel_msg = Twist()
        vel_msg.linear.x = 0.5  # 0.5 m/s forward
        
        start_time = time.time()
        duration = 3.0  # seconds
        
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(vel_msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Stop the robot
        vel_msg.linear.x = 0.0
        self.cmd_vel_pub.publish(vel_msg)
        
        # Reset steering
        steer_msg.data = 0.0
        self.left_steer_pub.publish(steer_msg)
        self.right_steer_pub.publish(steer_msg)
        
        # Verify movement
        if self.current_pose is not None and self.initial_pose is not None:
            dx = self.current_pose.position.x - self.initial_pose.position.x
            dy = self.current_pose.position.y - self.initial_pose.position.y
            distance = (dx**2 + dy**2)**0.5
            
            self.get_logger().info(f'Robot moved: {distance:.2f} meters')
            if distance > 0.1:  # We expect some movement
                self.get_logger().info('Test PASSED: Robot moved as expected')
                return True
            else:
                self.get_logger().warn('Test FAILED: Robot did not move as expected')
                return False
        else:
            self.get_logger().error('Test FAILED: Could not verify movement')
            return False

def main(args=None):
    rclpy.init(args=args)
    tester = GazeboSimTester()
    
    try:
        result = tester.run_test()
        if result:
            print("\n=== GAZEBO SIMULATION TEST PASSED ===\n")
        else:
            print("\n=== GAZEBO SIMULATION TEST FAILED ===\n")
    except Exception as e:
        tester.get_logger().error(f'Test failed with exception: {str(e)}')
        print("\n=== GAZEBO SIMULATION TEST FAILED ===\n")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 