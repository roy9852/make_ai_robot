#!/usr/bin/env python3
"""
ROS2 odom localizer node
This node is responsible for localizing the robot in the odom frame.
It uses various sensors to localize the robot in the odom frame.
To overcome the limitations of drift, noise, and other factors, global localiza
tion is used to correct the odom localization.
odom_localizer: tf from odom to base_link frame
global_localizer: tf from map to odom frame
combined: tf from map to base_link frame
Current code uses Iterative Closest Point (ICP) to localize the robot in the odom frame.
You can modify this node to use other sensors to localize the robot.
Usually, LiDAR is used to localize the robot in the map frame, for global loc
alization with given map.
Odom is usually done by cmd_vel, IMU, RGBD camera, etc.
But for simplicity, we use ICP to localize the robot in the odom frame.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
import tf_transformations
import numpy as np
from utils import scan_to_pcd, icp_2d

class OdomLocalizerNode(Node):
    def __init__(self):
        # Initialize the ROS2 node
        super().__init__('odom_localizer')
        self.get_logger().info('Odom localizer node initialized')

        # Create a subscription for the laser scan topic
        # The laser scan topic is the input point cloud for ICP registration.
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Create a tf publisher
        # This publisher is activated acsyncronously with the scan callback.
        self.tf_broadcaster = TransformBroadcaster(self)

        # Iterative Closest Point (ICP) works by comparing previous and current point cloud data (pcd).
        # Therefore, we need to store the previous and current pcd.
        # pcd data is a 2d point cloud of numpy array.
        # Each pcd is numpy array of (x, y) in the odom frame at previous time or current time.
        self.previous_pcd = None
        self.current_pcd = None

        # Parameters for ICP registration
        self.max_iterations = 10
        self.tolerance = 1e-6
        self.distance_threshold = 0.2

        # Current pose in the odom frame
        # pose is 3 x 3 SE(2) transformation matrix: [R, t; 0, 1] from odom frame to base frame.
        self.current_pose = np.eye(3)

    """
    Callback function for the laser scan topic.
    Update the current scan and localize the robot in the odom frame.
    """
    def scan_callback(self, msg):
        start_time = self.get_clock().now()
        # Update previous_pcd at the first subscription.
        # pcd is numpy array of (x, y) whose shape is (N, 2) in the odom frame at previous time.
        if self.previous_pcd is None:
            self.previous_pcd = scan_to_pcd(msg)
            return

        # Update current pcd from the second subscription.
        self.current_pcd = scan_to_pcd(msg)

        # Localize the robot in the odom frame with ICP
        # pose_delta is 3 x 3 SE(2) transformation matrix: [R, t; 0, 1] from previous base frame to current base frame.
        self.pose_delta = icp_2d(self.previous_pcd,
                                 self.current_pcd,
                                 self.max_iterations,
                                 self.tolerance,
                                 self.distance_threshold
                                )
        # Update previous pcd for the next iteration
        self.previous_pcd = self.current_pcd

        # Update current pose
        # current_pose: odom->previous_base_frame
        # pose_delta: previous_base_frame->current_base_frame
        # self.current_pose: odom->current_base_frame
        self.current_pose = self.current_pose @ self.pose_delta

        # Publish the current pose
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base'
        t.transform.translation.x = float(self.current_pose[0, 2])
        t.transform.translation.y = float(self.current_pose[1, 2])
        t.transform.translation.z = 0.0
        yaw = np.arctan2(self.current_pose[1, 0], self.current_pose[0, 0])
        q = tf_transformations.quaternion_from_euler(0, 0, float(yaw))
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        self.tf_broadcaster.sendTransform(t)
        end_time = self.get_clock().now()
        time_delta = (end_time - start_time).nanoseconds / 1e9
        frequency = 1.0 / time_delta
        self.get_logger().info(f'ICP registration frequency: {frequency:.3f} Hz')

if __name__ == '__main__':
    rclpy.init()
    node = OdomLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
