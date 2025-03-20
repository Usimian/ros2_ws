#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time

class WheelDriver(Node):
    def __init__(self):
        super().__init__('wheel_driver')
        
        # Publishers for wheel velocity commands
        self.fl_wheel_pub = self.create_publisher(
            Float64, 
            '/front_left_wheel_joint/velocity', 
            10
        )
        
        self.fr_wheel_pub = self.create_publisher(
            Float64, 
            '/front_right_wheel_joint/velocity', 
            10
        )
        
        self.rl_wheel_pub = self.create_publisher(
            Float64, 
            '/rear_left_wheel_joint/velocity', 
            10
        )
        
        self.rr_wheel_pub = self.create_publisher(
            Float64, 
            '/rear_right_wheel_joint/velocity', 
            10
        )
        
        # Timer for test sequence
        self.timer = self.create_timer(0.1, self.drive_wheels)
        self.test_duration = 0
        
        self.get_logger().info('Wheel driver initialized')
        
    def drive_wheels(self):
        # Create velocity commands
        velocity = Float64()
        velocity.data = 10.0  # Rad/sec, fairly fast rotation
        
        # Publish to all wheels
        self.fl_wheel_pub.publish(velocity)
        self.fr_wheel_pub.publish(velocity)
        self.rl_wheel_pub.publish(velocity)
        self.rr_wheel_pub.publish(velocity)
        
        self.test_duration += 1
        
        # Log every second
        if self.test_duration % 10 == 0:
            self.get_logger().info(f'Driving wheels at velocity: {velocity.data} rad/sec')
        
        # Stop after 10 seconds
        if self.test_duration > 100:
            velocity.data = 0.0
            self.fl_wheel_pub.publish(velocity)
            self.fr_wheel_pub.publish(velocity)
            self.rl_wheel_pub.publish(velocity)
            self.rr_wheel_pub.publish(velocity)
            self.get_logger().info('Test complete, stopping wheels')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    driver = WheelDriver()
    
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 