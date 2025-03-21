#!/usr/bin/env python3

import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float64, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import random

class AckermannController(Node):
    def __init__(self):
        super().__init__('ackermann_controller')
        
        # Publisher for velocity commands
        self.vel_pub = self.create_publisher(Twist, '/model/ackermann_robot/cmd_vel', 10)
        
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
            '/model/ackermann_robot/cmd_vel',
            self.cmd_vel_callback,
            10)
            
        # Enable/disable autonomous mode
        self.auto_mode_sub = self.create_subscription(
            Bool,
            'auto_mode',
            self.auto_mode_callback,
            10)
        
        # Initialize variables
        self.bridge = CvBridge()
        self.obstacle_detected = False
        self.min_distance = float('inf')
        self.current_angular_z = 0.0

        self.last_cmd_time = self.get_clock().now()
        self.auto_mode = False
        self.obstacle_directions = []  # Will store laser scan data for obstacle directions
        
        # Ackermann parameters
        self.wheel_base = 0.3  # Distance between front and rear axles
        self.wheel_track = 0.26  # Distance between left and right wheels
        self.max_steering_angle = 0.5  # Maximum steering angle in radians
        
        # Auto navigation parameters
        self.forward_speed = 1.0  # Default forward speed in auto mode (reduced for safety)
        self.turn_speed = 1.0     # Default turning speed in auto mode (increased for better turning)
        self.obstacle_threshold = 1.2  # Distance threshold for obstacle detection (increased for earlier detection)
        self.min_safe_distance = 0.5   # Minimum safe distance to obstacles
        self.critical_distance = 0.3   # Critical distance where robot will back up
        self.turn_direction = 1.0  # Default turn direction (1.0 = left, -1.0 = right)
        self.auto_state = "FORWARD"  # Current state in auto mode
        self.state_start_time = None  # To track time spent in each state
        self.state_distance_traveled = 0.0  # To track distance traveled in backing up state
        self.last_scan_time = None   # To track when we last received scan data
        
        # Debug variables
        self.camera_count = 0
        self.ultrasonic_count = 0
        self.stuck_timer = None  # To detect if we're stuck
        
        self.get_logger().info('Ackermann controller initialized')
    
    def auto_mode_callback(self, msg):
        # Toggle auto mode on/off
        if msg.data != self.auto_mode:
            self.auto_mode = msg.data
            if self.auto_mode:
                self.get_logger().info('Automatic obstacle avoidance mode ENABLED')
                self.auto_state = "FORWARD"
                self.state_start_time = self.get_clock().now()
                self.stuck_timer = None
            else:
                self.get_logger().info('Automatic obstacle avoidance mode DISABLED')
                # Stop the robot when switching off auto mode
                self.stop_robot()
    
    def camera_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Increment camera message counter
            self.camera_count += 1
            
            # Log that we received an image
            if self.camera_count % 50 == 0:  # Log every 10th image to reduce spam
                self.get_logger().info(f'Received camera image #{self.camera_count}: {cv_image.shape}')
            
            # Process the image here if needed
            # For example, detect lane lines, obstacles, etc.
            
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {str(e)}')
    
    def ultrasonic_callback(self, msg):
        # Increment ultrasonic message counter
        self.ultrasonic_count += 1
        self.last_scan_time = self.get_clock().now()
        
        # Get the minimum distance reading from the ultrasonic sensor
        if len(msg.ranges) > 0:
            # Store valid ranges for direction detection
            self.obstacle_directions = []
            valid_ranges = [r for r in msg.ranges if r > 0.0]
            if valid_ranges:
                self.min_distance = min(valid_ranges)
                
                # Store directions where obstacles are detected
                for i, r in enumerate(msg.ranges):
                    if 0 < r < self.obstacle_threshold:
                        angle = msg.angle_min + i * msg.angle_increment
                        self.obstacle_directions.append((angle, r))
                
                # Check if obstacle is detected
                if self.min_distance < self.obstacle_threshold:
                    self.obstacle_detected = True
                    if self.min_distance < self.min_safe_distance:
                        self.get_logger().warn(f'CRITICAL obstacle detected at {self.min_distance:.2f} meters')
                    else:
                        self.get_logger().info(f'Obstacle detected at {self.min_distance:.2f} meters')
                else:
                    self.obstacle_detected = False
                    
                # Log ultrasonic data periodically
                if self.ultrasonic_count % 10 == 0:  # Log every 10th reading
                    self.get_logger().info(f'Ultrasonic reading #{self.ultrasonic_count}: min_distance={self.min_distance:.2f}m, directions={len(self.obstacle_directions)}')
    
    def cmd_vel_callback(self, msg):
        # Ignore external commands if in auto mode
        if self.auto_mode:
            return
            
        # Store the angular velocity for steering calculation
        self.current_angular_z = msg.angular.z
        self.last_cmd_time = self.get_clock().now()
        
        # Calculate steering angle based on angular velocity
        # This is a simplified Ackermann steering model
        steering_angle = self.calculate_steering_angle(msg.linear.x, msg.angular.z)
        
        # Publish steering commands
        self.publish_steering_commands(steering_angle)
        
        self.get_logger().debug(f'Received cmd_vel: linear.x={msg.linear.x}, angular.z={msg.angular.z}, steering_angle={steering_angle}')
    
    def calculate_steering_angle(self, linear_x, angular_z):
        # Avoid division by zero
        if abs(linear_x) < 0.001 or abs(angular_z) < 0.001:
            return 0.0
            
        # Calculate the turning radius
        # For a differential drive, turning radius = linear_speed / angular_speed
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
    
    def navigation_control(self):
        # Handle both safety and auto navigation
        if self.auto_mode:
            self.auto_navigation()
        else:
            self.safety_check()
    
    def safety_check(self):
        # Only perform obstacle avoidance if needed
        if self.obstacle_detected:
            # Log the detected obstacle
            self.get_logger().warn(f'Obstacle detected at {self.min_distance:.2f}m - Safety system active')
            
            # Emergency stop could be implemented here if needed
            # For now, we just let the teleop control continue but log warnings
        else:
            # Just log the current state occasionally for debugging
            if (self.get_clock().now().to_msg().sec) % 10 == 0:  # Log every ~10 seconds
                self.get_logger().debug(f'Safety system monitoring: no obstacles detected, min_distance={self.min_distance:.2f}m')
    
    def auto_navigation(self):
        # Check if we're getting laser scan data
        # if self.last_scan_time is None:
        #     self.get_logger().warn('No laser scan data received yet. Waiting...')
        #     return
            
        # Implement a simple state machine for obstacle avoidance
        current_time = self.get_clock().now()
        timestamp = current_time.to_msg()
        # First check for critical distance - this overrides any current state
        if self.obstacle_detected and self.min_distance < self.critical_distance and self.auto_state != "BACKING_UP":
            self.get_logger().warn(f'TOO CLOSE to obstacle ({self.min_distance:.2f}m) - BACKING UP!')
            self.auto_state = "BACKING_UP"
            self.state_start_time = current_time
            self.state_distance_traveled = 0.0
            # Immediately stop before backing up
            self.stop_robot()
            return
            
        if self.auto_state == "FORWARD":
            # Go forward until we detect an obstacle
            if self.obstacle_detected and self.min_distance < self.obstacle_threshold:
                self.get_logger().info(f'Obstacle ahead at {self.min_distance:.2f}m - Starting avoidance maneuver')
                # Determine turn direction based on obstacle position
                self.choose_turn_direction()
                self.auto_state = "TURNING"
                self.state_start_time = current_time
                # Slow down or stop briefly before turning
                self.send_velocity_command(0.0, 0.0)
            else:
                # Keep going forward
                self.send_velocity_command(self.forward_speed, 0.0)
                
        elif self.auto_state == "TURNING":
            # Turn to avoid obstacle
            turn_duration = 3.0  # seconds (increased for more complete turns)
            current_sec = current_time.to_msg().sec + current_time.to_msg().nanosec / 1e9
            start_sec = self.state_start_time.to_msg().sec + self.state_start_time.to_msg().nanosec / 1e9
            time_diff = current_sec - start_sec
            
            if time_diff > turn_duration:
                # Done turning, go back to forward
                self.auto_state = "FORWARD"
                self.state_start_time = current_time
                self.get_logger().info('Completed turn, resuming forward motion')
            elif not self.obstacle_detected or self.min_distance > self.obstacle_threshold + 0.5:
                # Obstacle no longer in threshold + margin, can go forward again
                self.auto_state = "FORWARD"
                self.state_start_time = current_time
                self.get_logger().info('Path clear, resuming forward motion')
            else:
                # Continue turning
                turn_rate = self.turn_direction * self.turn_speed
                # Make sharper turns with less forward motion for better obstacle avoidance
                self.send_velocity_command(0.1, turn_rate)
                
        elif self.auto_state == "BACKING_UP":
            # Back up for about 1 meter
            backup_speed = -0.2  # Negative for reverse
            backup_time = 5.0    # seconds - adjust based on speed to get ~1 meter
            current_sec = current_time.to_msg().sec + current_time.to_msg().nanosec / 1e9
            start_sec = self.state_start_time.to_msg().sec + self.state_start_time.to_msg().nanosec / 1e9
            time_diff = current_sec - start_sec
            
            # Calculate approximate distance traveled (time * speed)
            self.state_distance_traveled = abs(backup_speed) * time_diff
            
            if time_diff > backup_time or self.state_distance_traveled >= 1.0:
                # Done backing up about 1 meter
                self.auto_state = "TURNING"
                self.state_start_time = current_time
                self.get_logger().info(f'Backed up {self.state_distance_traveled:.2f}m, now turning to avoid obstacle')
                # Choose direction to turn after backing up
                self.choose_turn_direction()
            else:
                # Continue backing up
                self.send_velocity_command(backup_speed, 0.0)
                self.get_logger().info(f'Backing up: {self.state_distance_traveled:.2f}m traveled')
                
        else:
            # Default to FORWARD if state is unknown
            self.auto_state = "FORWARD"
            self.state_start_time = current_time
            self.get_logger().warn('Unknown state, resetting to FORWARD')
    
    def choose_turn_direction(self):
        # Use laser scan data to determine best turn direction
        if not self.obstacle_directions:
            # No obstacle direction data, choose random
            self.turn_direction = 1.0 if random.random() > 0.5 else -1.0
            self.get_logger().info(f'No directional data, turning {"LEFT" if self.turn_direction > 0 else "RIGHT"} randomly')
            return
            
        # Check if obstacles more on left or right
        left_obstacles = sum(1 for angle, _ in self.obstacle_directions if angle > 0)
        right_obstacles = sum(1 for angle, _ in self.obstacle_directions if angle < 0)
        
        # Turn away from side with more obstacles
        if left_obstacles > right_obstacles:
            self.turn_direction = -1.0  # Turn right
            self.get_logger().info(f'Turning RIGHT to avoid obstacles (left: {left_obstacles}, right: {right_obstacles})')
        else:
            self.turn_direction = 1.0   # Turn left
            self.get_logger().info(f'Turning LEFT to avoid obstacles (left: {left_obstacles}, right: {right_obstacles})')
    
    def send_velocity_command(self, linear_x, angular_z):
        # Create and send Twist command
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        
        # Handle case where we're trying to turn in place
        if abs(linear_x) < 0.001 and abs(angular_z) > 0.001:
            # Use a small non-zero linear velocity for Ackermann turning
            cmd.linear.x = 0.05
        
        # Calculate steering angle based on velocities
        if abs(angular_z) < 0.001:
            steering_angle = 0.0
        else:
            # Direct calculation for when angular_z is present
            if angular_z > 0:  # turning left
                steering_angle = min(angular_z * 0.8, self.max_steering_angle)
            else:  # turning right
                steering_angle = max(angular_z * 0.8, -self.max_steering_angle)
        
        # Publish commands
        self.vel_pub.publish(cmd)
        self.publish_steering_commands(steering_angle)
    
    def stop_robot(self):
        # Emergency stop - send zero velocity
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.vel_pub.publish(cmd)
        
        # Center the steering
        self.publish_steering_commands(0.0)
        self.get_logger().info('Robot stopped')

def main(args=None):
    rclpy.init(args=args)
    controller = AckermannController()
    
    try:
        # Use a custom spin function to run navigation_control in the main loop
        while rclpy.ok():
            rclpy.spin_once(controller, timeout_sec=0.05)  # Process callbacks with 50ms timeout
            controller.navigation_control()  # Run navigation control in main loop
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 