#!/usr/bin/env python3

import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class AckermannController(Node):
    def __init__(self):
        super().__init__('ackermann_controller')
        
        # Publisher for velocity commands
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Publishers for steering commands
        self.left_steer_pub = self.create_publisher(
            Float64, 
            'front_left_steer_joint/cmd_position', 
            10
        )
        
        self.right_steer_pub = self.create_publisher(
            Float64, 
            'front_right_steer_joint/cmd_position', 
            10
        )
        
        # Subscribers for sensors
        self.camera_sub = self.create_subscription(
            Image,
            'camera',
            self.camera_callback,
            10)
        
        self.ultrasonic_sub = self.create_subscription(
            LaserScan,
            'ultrasonic',
            self.ultrasonic_callback,
            10)
        
        # Subscriber for cmd_vel to control steering
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Initialize variables
        self.bridge = CvBridge()
        self.obstacle_detected = False
        self.min_distance = float('inf')
        self.current_angular_z = 0.0
        
        # Ackermann parameters
        self.wheel_base = 0.3  # Distance between front and rear axles
        self.wheel_track = 0.26  # Distance between left and right wheels
        self.max_steering_angle = 0.5  # Maximum steering angle in radians
        
        # Timer for publishing commands
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Debug variables
        self.camera_count = 0
        self.ultrasonic_count = 0
        
        self.get_logger().info('Ackermann controller initialized')
    
    def camera_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Increment camera message counter
            self.camera_count += 1
            
            # Log that we received an image
            if self.camera_count % 10 == 0:  # Log every 10th image to reduce spam
                self.get_logger().info(f'Received camera image #{self.camera_count}: {cv_image.shape}')
            
            # Process the image here if needed
            # For example, detect lane lines, obstacles, etc.
            
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {str(e)}')
    
    def ultrasonic_callback(self, msg):
        # Increment ultrasonic message counter
        self.ultrasonic_count += 1
        
        # Get the minimum distance reading from the ultrasonic sensor
        if len(msg.ranges) > 0:
            self.min_distance = min(r for r in msg.ranges if r > 0.0)
            
            # Check if obstacle is detected
            if self.min_distance < 1.0:  # 1 meter threshold
                self.obstacle_detected = True
                self.get_logger().info(f'Obstacle detected at {self.min_distance:.2f} meters')
            else:
                self.obstacle_detected = False
                
            # Log ultrasonic data periodically
            if self.ultrasonic_count % 10 == 0:  # Log every 10th reading
                self.get_logger().info(f'Ultrasonic reading #{self.ultrasonic_count}: min_distance={self.min_distance:.2f}m, ranges={msg.ranges}')
    
    def cmd_vel_callback(self, msg):
        # Store the angular velocity for steering calculation
        self.current_angular_z = msg.angular.z
        
        # Calculate steering angle based on angular velocity
        # This is a simplified Ackermann steering model
        steering_angle = self.calculate_steering_angle(msg.linear.x, msg.angular.z)
        
        # Publish steering commands
        self.publish_steering_commands(steering_angle)
        
        self.get_logger().debug(f'Received cmd_vel: linear.x={msg.linear.x}, angular.z={msg.angular.z}, steering_angle={steering_angle}')
    
    def calculate_steering_angle(self, linear_x, angular_z):
        # Avoid division by zero
        if abs(linear_x) < 0.001:
            return 0.0
        
        # Calculate the turning radius
        # For a differential drive, turning radius = linear_speed / angular_speed
        if abs(angular_z) < 0.001:
            return 0.0
            
        turning_radius = linear_x / angular_z
        
        # Calculate the steering angle using Ackermann formula
        # tan(steering_angle) = wheelbase / turning_radius
        steering_angle = math.atan(self.wheel_base / turning_radius)
        
        # Limit the steering angle to the maximum
        steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle))
        
        return steering_angle
    
    def publish_steering_commands(self, steering_angle):
        # For Ackermann steering, the inner wheel turns more than the outer wheel
        # This is a simplified model - in a real Ackermann system, the angles would be calculated
        # based on the Ackermann geometry
        
        # Determine left and right steering angles
        if steering_angle > 0:  # Turning left
            left_angle = steering_angle
            right_angle = math.atan(self.wheel_base / (self.wheel_base / math.tan(steering_angle) + self.wheel_track))
        elif steering_angle < 0:  # Turning right
            right_angle = steering_angle
            left_angle = math.atan(self.wheel_base / (self.wheel_base / math.tan(steering_angle) - self.wheel_track))
        else:  # Going straight
            left_angle = 0.0
            right_angle = 0.0
        
        # Publish the steering commands
        left_cmd = Float64()
        left_cmd.data = left_angle
        self.left_steer_pub.publish(left_cmd)
        
        right_cmd = Float64()
        right_cmd.data = right_angle
        self.right_steer_pub.publish(right_cmd)
    
    def control_loop(self):
        # Basic obstacle avoidance
        if self.obstacle_detected:
            # Stop and turn
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # Turn left
            self.vel_pub.publish(twist)
            
            # Update steering based on the new angular velocity
            steering_angle = self.calculate_steering_angle(0.0, 0.5)
            self.publish_steering_commands(steering_angle)
            
            self.get_logger().debug('Obstacle avoidance: turning left')
        else:
            # If no obstacle is detected and no command has been received recently,
            # publish a small forward velocity to test movement
            twist = Twist()
            twist.linear.x = 1.0  # Increased forward movement
            twist.angular.z = 0.0
            self.vel_pub.publish(twist)
            
            # Update steering for straight movement
            steering_angle = self.calculate_steering_angle(1.0, 0.0)
            self.publish_steering_commands(steering_angle)
            
            self.get_logger().info('Publishing test movement command: linear.x=1.0, angular.z=0.0')
            
            # Log the current state
            self.get_logger().info(f'Current state: obstacle_detected={self.obstacle_detected}, min_distance={self.min_distance:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    controller = AckermannController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 