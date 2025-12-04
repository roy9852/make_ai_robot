#!/usr/bin/env python3

"""
ROS2 global localizer node

This node is responsible for localizing the robot in the global frame.
It publishes tf topic from map to odom frame.
By combining the odom localization and global localization, we can get a accurate localization.

odom_localizer: tf from odom to base frame
global_localizer: tf from map to odom frame
combined: tf from map to base frame

Current code only publishes the initial pose from launch file as odom frame.
You need to modify this node to use map and other sensors to localize the robot.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf_transformations
from tf2_ros import TransformBroadcaster, Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from rclpy.time import Time
from rosgraph_msgs.msg import Clock

from utils import pose_to_matrix, transform_to_matrix

import numpy as np

class GlobalLocalizerNode(Node):
    def __init__(self):
        # Initialize the ROS2 node
        super().__init__('global_localizer')       
        self.get_logger().info('Global localizer node initialized')

        # Create a subscription to the clock topic
        # By default, ROS2 uses system clock. But we use simulation clock from Gazebo instead.
        # This clock will be utilized when publishing the tf from map to odom frame.
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        self.current_time = None

        # Create TF buffer and listener to receive transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a timer to publish the tf from map to odom frame
        # This timer will be used to publish the tf from map to odom frame at a fixed interval
        self.interval_tf_pub = 0.1
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_timer = self.create_timer(self.interval_tf_pub, self.tf_callback) # publish the tf from map to odom frame at a fixed interval

        # Get the initial pose from the launch file
        # Launch file will set the initial pose of the robot in the map frame (x, y, yaw)
        # We convert the initial pose to the format of (x, y, z, qx, qy, qz, qw) for tf_transformations
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 1.0)
        self.declare_parameter('yaw', 0.0)
        self.x = self.get_parameter('x').value
        self.y = self.get_parameter('y').value
        self.yaw = self.get_parameter('yaw').value
        self.z = 0.33 # height of the robot base when robot is standing (z-axis is up)
        q = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        self.T_map_to_base = pose_to_matrix([self.x, self.y, self.z, q[0], q[1], q[2], q[3]])
        self.T_map_to_odom = None

    def clock_callback(self, msg):
        """
        Callback function for the clock topic. Update simulation time.
        """
        self.current_time = Time.from_msg(msg.clock)

    def tf_callback(self):
        """
        1. Subscribe to the tf from odom to base frame
        2. Calculate the tf from map to odom frame.
        3. Publish the tf from map to odom frame.
        """
        if self.current_time is None:
            return

        if self.T_map_to_odom is None:
            try:
                # Subscribe to the tf from odom to base frame
                odom_to_base_tf = self.tf_buffer.lookup_transform('odom', 'base', rclpy.time.Time())
                
                # T_odom_to_base (from tf lookup)
                self.T_odom_to_base = transform_to_matrix(odom_to_base_tf.transform)
                
                # Calculate T_map_to_odom = T_map_to_base * inverse(T_odom_to_base)
                # This is because: T_map_to_base = T_map_to_odom * T_odom_to_base
                # Therefore: T_map_to_odom = T_map_to_base * T_odom_to_base^(-1)
                self.T_base_to_odom = np.linalg.inv(self.T_odom_to_base)
                self.T_map_to_odom = self.T_map_to_base @ self.T_base_to_odom
                
                # Extract translation and rotation from the result
                translation = self.T_map_to_odom[:3, 3]
                quaternion = tf_transformations.quaternion_from_matrix(self.T_map_to_odom)
                
                # Publish the tf from map to odom frame
                tf_msg = TransformStamped()
                tf_msg.header.stamp = self.current_time.to_msg()
                tf_msg.header.frame_id = 'map'
                tf_msg.child_frame_id = 'odom'
                tf_msg.transform.translation.x = translation[0]
                tf_msg.transform.translation.y = translation[1]
                tf_msg.transform.translation.z = translation[2]
                tf_msg.transform.rotation.x = quaternion[0]
                tf_msg.transform.rotation.y = quaternion[1]
                tf_msg.transform.rotation.z = quaternion[2]
                tf_msg.transform.rotation.w = quaternion[3]
                self.tf_broadcaster.sendTransform(tf_msg)
                
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(f'Could not get transform from odom to base: {str(e)}')
                return
        else:
            # Extract translation and rotation from the result
            translation = self.T_map_to_odom[:3, 3]
            quaternion = tf_transformations.quaternion_from_matrix(self.T_map_to_odom)            

            # Publish the tf from map to odom frame
            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.current_time.to_msg()
            tf_msg.header.frame_id = 'map'
            tf_msg.child_frame_id = 'odom'
            tf_msg.transform.translation.x = translation[0]
            tf_msg.transform.translation.y = translation[1]
            tf_msg.transform.translation.z = translation[2]
            tf_msg.transform.rotation.x = quaternion[0]
            tf_msg.transform.rotation.y = quaternion[1]
            tf_msg.transform.rotation.z = quaternion[2]
            tf_msg.transform.rotation.w = quaternion[3]
            self.tf_broadcaster.sendTransform(tf_msg)            
            

if __name__ == '__main__':
    rclpy.init()
    node = GlobalLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()