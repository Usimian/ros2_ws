#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import threading
import sys
import termios
import tty

class AutoModeToggle(Node):
    def __init__(self):
        super().__init__('auto_mode_toggle')
        
        # Publisher for auto mode
        self.auto_mode_pub = self.create_publisher(Bool, 'auto_mode', 10)
        
        # Status
        self.auto_mode_enabled = False
        
        # Info to user
        self.get_logger().info('Auto Mode Toggle initialized')
        self.get_logger().info('Press "a" to toggle autonomous mode on/off')
        self.get_logger().info('Press "q" to quit')
        
        # Start key listening thread
        self.running = True
        self.key_thread = threading.Thread(target=self.read_keys)
        self.key_thread.daemon = True
        self.key_thread.start()
    
    def read_keys(self):
        # Save terminal settings
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            # Set terminal to raw mode
            tty.setraw(sys.stdin.fileno())
            while self.running:
                # Read a key
                key = sys.stdin.read(1)
                
                # Process the key
                if key == 'a':
                    self.toggle_auto_mode()
                elif key == 'q':
                    self.get_logger().info('Quitting...')
                    self.running = False
                    break
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
    def toggle_auto_mode(self):
        # Toggle auto mode
        self.auto_mode_enabled = not self.auto_mode_enabled
        
        # Create and publish message
        msg = Bool()
        msg.data = self.auto_mode_enabled
        self.auto_mode_pub.publish(msg)
        
        # Log the change
        if self.auto_mode_enabled:
            self.get_logger().info('Autonomous mode ENABLED')
        else:
            self.get_logger().info('Autonomous mode DISABLED')

def main(args=None):
    rclpy.init(args=args)
    toggle = AutoModeToggle()
    
    try:
        rclpy.spin(toggle)
    except KeyboardInterrupt:
        pass
    finally:
        toggle.running = False
        toggle.key_thread.join(timeout=1.0)
        toggle.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 