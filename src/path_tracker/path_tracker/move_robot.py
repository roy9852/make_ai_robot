#! /usr/bin/env python3

"""
Example Python node for path tracker
Depending on the argument number, generate path and move the robot
number 1: straight line of 2m
number 2: square of 2m (straight line of 2m, turn right, straight line of 2m, and so on. Final orientation should be same as the initial orientation)
number 3: circle of 2m (center is at right 1m)
other: log error and exit
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Twist
from nav_msgs.msg import Path
import math
import sys
import numpy as np
import time


class PathTrackerExample(Node):
    def __init__(self, target_x, target_y, target_yaw):
        super().__init__('path_tracker_example')
        
        self.target_x = target_x
        self.target_y = target_y
        self.target_yaw = target_yaw
        
        self.robot_pose = None
        self.initial_pose = None
        self.path_published = False
        self.goal_position = None
        
        # Velocity tracking
        self.current_cmd_vel = None
        self.zero_velocity_count = 0
        self.goal_reached = False
        
        # Subscribe to robot pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/go1_pose',
            self.pose_callback,
            10
        )
        
        # Subscribe to cmd_vel to monitor velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Subscribe to tracking info (cross-track error, target speed, current speed)
        self.tracking_info_sub = self.create_subscription(
            Point,
            '/path_tracker/tracking_info',
            self.tracking_info_callback,
            10
        )
        
        # Publisher for path
        self.path_pub = self.create_publisher(
            Path,
            '/local_path',
            10
        )
        
        # Timer to check if we should publish path
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        # Timer to check for goal arrival
        self.goal_check_timer = self.create_timer(1.0, self.check_goal_arrival)
        
        self.get_logger().info(f'Path Tracker Example initialized')
        self.get_logger().info(f'Target: x={target_x:.3f}m, y={target_y:.3f}m, yaw={target_yaw:.3f}rad ({math.degrees(target_yaw):.1f}°)')
        
    def pose_callback(self, msg):
        """Callback for robot pose"""
        self.robot_pose = msg
        
        # Store initial pose when first received
        if self.initial_pose is None:
            self.initial_pose = msg
            self.get_logger().info(f'Initial pose received: x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}')
    
    def cmd_vel_callback(self, msg):
        """Callback for velocity commands"""
        self.current_cmd_vel = msg
        
        # Log velocity commands
        if self.path_published and not self.goal_reached:
            self.get_logger().info(
                f'Cmd Vel: linear_x={msg.linear.x:.3f} m/s, angular_z={msg.angular.z:.3f} rad/s',
                throttle_duration_sec=2.0
            )
    
    def tracking_info_callback(self, msg):
        """Callback for tracking information (cross-track error, target speed, current speed)"""
        cte = msg.x
        target_speed = msg.y
        current_speed = msg.z
        
        if self.path_published and not self.goal_reached:
            self.get_logger().info(
                f'Tracking: CTE={cte:.3f}m, Target={target_speed:.3f}m/s, Current={current_speed:.3f}m/s',
                throttle_duration_sec=2.0
            )
    
    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle"""
        q = quaternion
        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion"""
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q
    
    def generate_smooth_path(self, start_pose, target_x, target_y, target_yaw, num_points=60):
        """Generate a smooth curved path from current position to target
        
        Uses a cubic Hermite spline that respects initial and final orientations.
        The path smoothly curves from start to target position.
        
        Args:
            start_pose: Current robot pose
            target_x: Target x position
            target_y: Target y position
            target_yaw: Target orientation (radians)
            num_points: Number of waypoints along the path
        """
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        
        # Start position and orientation
        x0 = start_pose.pose.position.x
        y0 = start_pose.pose.position.y
        z0 = start_pose.pose.position.z
        yaw0 = self.quaternion_to_yaw(start_pose.pose.orientation)
        
        # Target position and orientation
        x1 = target_x
        y1 = target_y
        z1 = 0.0  # Keep z at ground level
        yaw1 = target_yaw
        
        # Distance to home
        distance = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
        
        # Control point scale (adjust for smoothness)
        # Longer distance = longer control arms
        control_scale = min(distance * 0.5, 2.0)
        
        # Start control point: extends in direction of initial orientation
        cx0 = x0 + control_scale * math.cos(yaw0)
        cy0 = y0 + control_scale * math.sin(yaw0)
        
        # End control point: comes from direction of final orientation
        cx1 = x1 - control_scale * math.cos(yaw1)
        cy1 = y1 - control_scale * math.sin(yaw1)
        
        # Generate smooth path using cubic Hermite interpolation
        for i in range(num_points + 1):
            t = i / num_points  # Parameter from 0 to 1
            
            # Cubic Hermite basis functions
            h00 = 2*t**3 - 3*t**2 + 1      # Start point
            h10 = t**3 - 2*t**2 + t         # Start tangent
            h01 = -2*t**3 + 3*t**2          # End point
            h11 = t**3 - t**2                # End tangent
            
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            
            # Interpolate position using Hermite spline
            # P(t) = h00*P0 + h10*m0 + h01*P1 + h11*m1
            # where m0 and m1 are tangent vectors
            tangent_x0 = (cx0 - x0)
            tangent_y0 = (cy0 - y0)
            tangent_x1 = (x1 - cx1)
            tangent_y1 = (y1 - cy1)
            
            pose_stamped.pose.position.x = (h00 * x0 + h10 * tangent_x0 + 
                                           h01 * x1 + h11 * tangent_x1)
            pose_stamped.pose.position.y = (h00 * y0 + h10 * tangent_y0 + 
                                           h01 * y1 + h11 * tangent_y1)
            pose_stamped.pose.position.z = z0 + t * (z1 - z0)  # Linear interpolation for z
            
            # Calculate orientation from path tangent
            # Derivative of Hermite spline gives tangent direction
            if i < num_points:
                # Look ahead to next point to compute tangent
                t_next = (i + 1) / num_points
                h00_next = 2*t_next**3 - 3*t_next**2 + 1
                h10_next = t_next**3 - 2*t_next**2 + t_next
                h01_next = -2*t_next**3 + 3*t_next**2
                h11_next = t_next**3 - t_next**2
                
                x_next = (h00_next * x0 + h10_next * tangent_x0 + 
                         h01_next * x1 + h11_next * tangent_x1)
                y_next = (h00_next * y0 + h10_next * tangent_y0 + 
                         h01_next * y1 + h11_next * tangent_y1)
                
                # Tangent direction
                dx = x_next - pose_stamped.pose.position.x
                dy = y_next - pose_stamped.pose.position.y
                tangent_yaw = math.atan2(dy, dx)
            else:
                # Last point uses final orientation
                tangent_yaw = yaw1
            
            pose_stamped.pose.orientation = self.yaw_to_quaternion(tangent_yaw)
            path.poses.append(pose_stamped)
        
        return path


    
    def timer_callback(self):
        """Timer callback to publish path once robot pose is available"""
        if self.path_published:
            return
            
        if self.initial_pose is None:
            self.get_logger().warn('Waiting for robot pose from /go1_pose...', throttle_duration_sec=2.0)
            return
        
        # Calculate distance to target
        x_start = self.initial_pose.pose.position.x
        y_start = self.initial_pose.pose.position.y
        distance = math.sqrt((self.target_x - x_start)**2 + (self.target_y - y_start)**2)
        
        # Determine number of points based on distance (more points for longer paths)
        num_points = max(40, min(80, int(distance * 15)))
        
        # Generate smooth path to target
        self.get_logger().info(f'Generating smooth curved path...')
        self.get_logger().info(f'  From: ({x_start:.3f}, {y_start:.3f})')
        self.get_logger().info(f'  To:   ({self.target_x:.3f}, {self.target_y:.3f})')
        self.get_logger().info(f'  Distance: {distance:.3f}m with {num_points} waypoints')
        
        path = self.generate_smooth_path(
            self.initial_pose, 
            self.target_x, 
            self.target_y, 
            self.target_yaw, 
            num_points=num_points
        )
        
        # Store goal position (last point in path)
        if len(path.poses) > 0:
            last_pose = path.poses[-1]
            self.goal_position = last_pose.pose.position
        
        # Publish the path
        self.path_pub.publish(path)
        self.get_logger().info(f'Path published with {len(path.poses)} points.')
        self.path_published = True
        
        # Log initial position for reference
        x0 = self.initial_pose.pose.position.x
        y0 = self.initial_pose.pose.position.y
        yaw0 = self.quaternion_to_yaw(self.initial_pose.pose.orientation)
        self.get_logger().info(f'Starting position: x={x0:.3f}, y={y0:.3f}, yaw={yaw0:.3f} rad ({math.degrees(yaw0):.1f}°)')
        
        if self.goal_position:
            self.get_logger().info(f'Goal position: x={self.goal_position.x:.3f}, y={self.goal_position.y:.3f}')
    
    def check_goal_arrival(self):
        """Check if robot has arrived at goal and shutdown if so"""
        if not self.path_published or self.goal_reached:
            return
        
        if self.robot_pose is None or self.goal_position is None:
            return
        
        # Calculate distance to goal
        dx = self.robot_pose.pose.position.x - self.goal_position.x
        dy = self.robot_pose.pose.position.y - self.goal_position.y
        distance_to_goal = math.sqrt(dx*dx + dy*dy)
        
        # Check if velocity is near zero
        if self.current_cmd_vel is not None:
            vel_magnitude = math.sqrt(
                self.current_cmd_vel.linear.x**2 + 
                self.current_cmd_vel.angular.z**2
            )
            
            # Goal reached if close to goal AND velocity is very low
            if distance_to_goal < 0.5 and vel_magnitude < 0.05:
                self.zero_velocity_count += 1
                
                # Need multiple consecutive checks to confirm arrival
                if self.zero_velocity_count >= 3:
                    self.goal_reached = True
                    self.get_logger().info('=' * 60)
                    self.get_logger().info('🎉 GOAL REACHED! 🎉')
                    self.get_logger().info(f'Final position: x={self.robot_pose.pose.position.x:.3f}, '
                                         f'y={self.robot_pose.pose.position.y:.3f}')
                    self.get_logger().info(f'Distance to goal: {distance_to_goal:.3f}m')
                    self.get_logger().info('Shutting down node...')
                    self.get_logger().info('=' * 60)
                    
                    # Shutdown after a short delay
                    time.sleep(2.0)
                    rclpy.shutdown()
            else:
                self.zero_velocity_count = 0


def main(args=None):
    # Check command line arguments
    if len(sys.argv) < 4:
        print("Usage: ros2 run path_tracker path_tracker_example.py <target_x> <target_y> <target_yaw>")
        print("")
        print("Arguments:")
        print("  target_x   : Target x position in meters")
        print("  target_y   : Target y position in meters")
        print("  target_yaw : Target orientation in radians")
        print("")
        print("Examples:")
        print("  # Move to (2.0, 1.0) facing east (0 rad)")
        print("  ros2 run path_tracker path_tracker_example.py 2.0 1.0 0.0")
        print("")
        print("  # Move to origin facing east")
        print("  ros2 run path_tracker path_tracker_example.py 0.0 0.0 0.0")
        print("")
        print("  # Move to (3.0, -2.0) facing northeast (pi/4 rad)")
        print("  ros2 run path_tracker path_tracker_example.py 3.0 -2.0 0.785")
        print("")
        print("  # Move to (-1.0, 2.0) facing south (-pi/2 rad)")
        print("  ros2 run path_tracker path_tracker_example.py -1.0 2.0 -1.57")
        sys.exit(1)
    
    try:
        target_x = float(sys.argv[1])
        target_y = float(sys.argv[2])
        target_yaw = float(sys.argv[3])
    except ValueError:
        print("Error: All arguments must be numbers")
        print("Usage: ros2 run path_tracker path_tracker_example.py <target_x> <target_y> <target_yaw>")
        sys.exit(1)
    
    rclpy.init(args=args)
    
    node = PathTrackerExample(target_x, target_y, target_yaw)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()