#! /usr/bin/env python3

"""
Move the Go1 robot with the keyboard.

Controls:
  w: forward
  x: backward
  s: stop
  a: rotate left
  d: rotate right
  q: quit

Based on these commands:
w: forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10

x: backward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: -0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10

s: stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10  

a: rotate left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}" -r 10

d: rotate right
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.2}}" -r 10
"""

import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class KeyboardController(Node):
    """
    ROS 2 node for controlling the Go1 robot with keyboard input.
    """

    def __init__(self):
        super().__init__('keyboard_controller')
        
        # Create publisher for velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Define key bindings
        self.key_bindings = {
            'w': {'linear_x': 0.2, 'angular_z': 0.0},    # Forward
            'x': {'linear_x': -0.2, 'angular_z': 0.0},   # Backward
            's': {'linear_x': 0.0, 'angular_z': 0.0},    # Stop
            'a': {'linear_x': 0.05, 'angular_z': 0.2},   # Rotate left
            'd': {'linear_x': 0.05, 'angular_z': -0.2},  # Rotate right
        }
        
        self.get_logger().info('Keyboard controller started')
        self.print_instructions()

    def print_instructions(self):
        """Print control instructions."""
        print("\n" + "="*50)
        print("Go1 Robot Keyboard Controller")
        print("="*50)
        print("Controls:")
        print("  w : Move forward")
        print("  x : Move backward")
        print("  s : Stop")
        print("  a : Rotate left")
        print("  d : Rotate right")
        print("  q : Quit")
        print("="*50 + "\n")

    def publish_velocity(self, linear_x, angular_z):
        """
        Publish velocity command to /cmd_vel topic.
        
        Args:
            linear_x: Linear velocity in x direction
            angular_z: Angular velocity around z axis
        """
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = angular_z
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: linear.x={linear_x}, angular.z={angular_z}')

    def get_key(self):
        """
        Read a single key from stdin without waiting for Enter.
        
        Returns:
            str: The pressed key
        """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        """Main loop to read keyboard input and publish commands."""
        try:
            while True:
                key = self.get_key()
                
                if key == 'q':
                    # Stop the robot before quitting
                    self.publish_velocity(0.0, 0.0)
                    self.get_logger().info('Quitting...')
                    break
                elif key in self.key_bindings:
                    # Publish corresponding velocity
                    binding = self.key_bindings[key]
                    self.publish_velocity(binding['linear_x'], binding['angular_z'])
                elif key == '\x03':  # Ctrl+C
                    # Stop the robot before quitting
                    self.publish_velocity(0.0, 0.0)
                    break
                    
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            # Ensure robot stops when exiting
            self.publish_velocity(0.0, 0.0)
            self.get_logger().info('Controller stopped')


def main(args=None):
    """
    Main function to initialize and run the keyboard controller.
    """
    rclpy.init(args=args)
    
    controller = KeyboardController()
    
    try:
        controller.run()
    except KeyboardInterrupt:
        controller.get_logger().info('Interrupted by user')
    finally:
        # Stop the robot
        controller.publish_velocity(0.0, 0.0)
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
