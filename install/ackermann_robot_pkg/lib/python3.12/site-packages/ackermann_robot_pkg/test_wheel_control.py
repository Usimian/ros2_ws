#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time

class WheelTester(Node):
    def __init__(self):
        super().__init__('wheel_tester')
        
        # Publishers for steering commands
        self.left_steer_pub = self.create_publisher(
            Float64, 
            '/front_left_steer_joint/cmd_position', 
            10
        )
        
        self.right_steer_pub = self.create_publisher(
            Float64, 
            '/front_right_steer_joint/cmd_position', 
            10
        )
        
        # Timer for test sequence
        self.timer = self.create_timer(1.0, self.test_sequence)
        self.test_step = 0
        
        self.get_logger().info('Wheel tester initialized')
        
    def test_sequence(self):
        if self.test_step == 0:
            # Turn wheels left
            self.get_logger().info('Testing: Turning wheels LEFT')
            left_cmd = Float64()
            left_cmd.data = 0.5  # 0.5 radians to the left
            self.left_steer_pub.publish(left_cmd)
            
            right_cmd = Float64()
            right_cmd.data = 0.3  # Less turn for the right wheel (Ackermann geometry)
            self.right_steer_pub.publish(right_cmd)
            
        elif self.test_step == 3:
            # Turn wheels right
            self.get_logger().info('Testing: Turning wheels RIGHT')
            left_cmd = Float64()
            left_cmd.data = -0.3  # -0.3 radians (right)
            self.left_steer_pub.publish(left_cmd)
            
            right_cmd = Float64()
            right_cmd.data = -0.5  # More turn for right wheel
            self.right_steer_pub.publish(right_cmd)
            
        elif self.test_step == 6:
            # Center wheels
            self.get_logger().info('Testing: Centering wheels')
            left_cmd = Float64()
            left_cmd.data = 0.0
            self.left_steer_pub.publish(left_cmd)
            
            right_cmd = Float64()
            right_cmd.data = 0.0
            self.right_steer_pub.publish(right_cmd)
            
        elif self.test_step > 8:
            # Restart the sequence
            self.test_step = -1
            
        self.test_step += 1

def main(args=None):
    rclpy.init(args=args)
    tester = WheelTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 