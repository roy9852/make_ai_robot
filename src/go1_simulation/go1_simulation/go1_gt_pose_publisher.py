#!/usr/bin/env python3

""" 
This script subscribe to the '/pose_from_gazebo' topic and publish the pose of the robot.
Use the first pose in the array as the pose of the robot.

By default, publishes geometry_msgs/msg/PoseStamped (3D pose) to '/go1_pose'.
With argument --2d_pose true, publishes geometry_msgs/msg/Pose2D (x, y, theta) to '/go1_pose_2d'.

Additionally, always publishes TF transform from 'world' to 'base' frame.

EXAMPLE MESSAGE FROM '/pose_from_gazebo' TOPIC
---
header:
  stamp:
    sec: 17
    nanosec: 922000000
  frame_id: ''
poses:
- position:
    x: 2.028990027703115
    y: 0.9825500261870626
    z: 0.04494268878911188
  orientation:
    x: 0.005638639478584912
    y: 0.006647860552908017
    z: 0.706209895210689
    w: 0.707948865103552
- position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
- position:
    x: 0.18810000000000016
    y: 0.04674999999999985
    z: 0.0
  orientation:
    x: 0.07273664246380611
    y: 6.938893903907228e-18
    z: 1.1102230246251565e-16
    w: 0.9973511823039529
- position:
    x: 0.18810000000000038
    y: 0.12590350093489455
    z: 0.011607036217295502
  orientation:
    x: 0.05965257089109227
    y: 0.5706799482094915
    z: 0.0416195860502091
    w: 0.8179448499469293
- position:
    x: -0.011807930870611938
    y: 0.1365710055488154
    z: -0.06113826610708381
  orientation:
    x: 0.050727335934576956
    y: -0.7147730483055801
    z: -0.0521282697781022
    w: 0.6955637068557207
- position:
    x: 0.18810000000000016
    y: -0.04675000000000029
    z: 0.0
  orientation:
    x: -0.0796594874078056
    y: -6.938893903907228e-18
    z: 0.0
    w: 0.9968221336155844
- position:
    x: 0.18810000000000038
    y: -0.1257346985705481
    z: 0.0127050144320916
  orientation:
    x: -0.06555691446378847
    y: 0.5662877647407083
    z: -0.045254004243402035
    w: 0.820349031553581
- position:
    x: -0.011063661547450754
    y: -0.13772777534589675
    z: -0.06185370087653726
  orientation:
    x: 0.05522435713619689
    y: 0.7184018837626943
    z: -0.05740996701765394
    w: -0.6910521684157702
- position:
    x: -0.1880999999999995
    y: 0.04675000000000029
    z: 0.0
  orientation:
    x: 0.020432344373107765
    y: -1.734723475976807e-18
    z: 5.551115123125783e-17
    w: 0.999791237860894
- position:
    x: -0.1880999999999996
    y: 0.1266832030885472
    z: 0.0032684926197103226
  orientation:
    x: 0.01681245339191595
    y: 0.5681613441742244
    z: 0.011611292241861693
    w: 0.8226634830169365
- position:
    x: -0.387297942769648
    y: 0.1297648462171348
    z: -0.0720951794774471
  orientation:
    x: 0.014167983498110881
    y: -0.7203932218069573
    z: -0.014722395870867211
    w: 0.6932648305503538
- position:
    x: -0.18809999999999993
    y: -0.04675000000000029
    z: 0.0
  orientation:
    x: -0.013159935716945221
    y: 8.673617379884035e-19
    z: 5.551115123125783e-17
    w: 0.9999134042965552
- position:
    x: -0.18809999999999938
    y: -0.1267222905747083
    z: 0.0021054073796887146
  orientation:
    x: -0.010825098487533769
    y: 0.5686003181333695
    z: -0.007483391664835304
    w: 0.822508659109773
- position:
    x: -0.38736562951153986
    y: -0.12870262755622575
    z: -0.07311617968685324
  orientation:
    x: 0.00912948453256962
    y: 0.7201696939189279
    z: -0.009478207649323889
    w: -0.6936731420868504
---

"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose2D, PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import math


class Go1GTPosePublisher(Node):
    """
    ROS 2 node that subscribes to ground truth poses from Gazebo and publishes
    the robot's pose to a separate topic.
    Supports both 3D pose (PoseStamped) and 2D pose (Pose2D) output modes.
    Also broadcasts TF transform from world to base frame.
    """

    def __init__(self, use_2d_pose=False):
        super().__init__('go1_gt_pose_publisher')
        
        # Store the pose mode
        self.use_2d_pose = use_2d_pose
        
        # Declare parameter
        self.declare_parameter('use_2d_pose', use_2d_pose)
        self.use_2d_pose = self.get_parameter('use_2d_pose').value
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create subscriber to the pose array from Gazebo
        self.subscription = self.create_subscription(
            PoseArray,
            '/pose_from_gazebo',
            self.pose_callback,
            10
        )
        
        # Create publisher based on the mode with different topic names
        if self.use_2d_pose:
            self.publisher = self.create_publisher(
                Pose2D,
                '/go1_pose_2d',
                10
            )
            self.get_logger().info('Ground truth 2D pose publisher node started')
            self.get_logger().info('Publishing to: /go1_pose_2d (Pose2D: x, y, theta)')
        else:
            self.publisher = self.create_publisher(
                PoseStamped,
                '/go1_pose',
                10
            )
            self.get_logger().info('Ground truth 3D pose publisher node started')
            self.get_logger().info('Publishing to: /go1_pose (PoseStamped: full 3D pose)')
        
        self.get_logger().info('Subscribing to: /pose_from_gazebo')
        self.get_logger().info('Broadcasting TF: world -> base')

    def quaternion_to_yaw(self, quat):
        """
        Convert quaternion to yaw angle (theta).
        
        Args:
            quat: Quaternion with x, y, z, w components
            
        Returns:
            float: Yaw angle in radians
        """
        # Convert quaternion to yaw using the formula:
        # yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def pose_callback(self, msg):
        """
        Callback function that processes incoming pose array messages.
        Extracts the first pose and publishes it as either PoseStamped or Pose2D.
        Also broadcasts the TF transform from world to base.
        
        Args:
            msg (PoseArray): Array of poses from Gazebo
        """
        # Check if the poses array is not empty
        if len(msg.poses) == 0:
            self.get_logger().warn('Received empty pose array')
            return
        
        # Get the first pose (robot's pose)
        robot_pose = msg.poses[0]
        
        # Broadcast TF transform from world to base (always)
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'base'
        
        # Set translation
        t.transform.translation.x = robot_pose.position.x
        t.transform.translation.y = robot_pose.position.y
        t.transform.translation.z = robot_pose.position.z
        
        # Set rotation (quaternion)
        t.transform.rotation.x = robot_pose.orientation.x
        t.transform.rotation.y = robot_pose.orientation.y
        t.transform.rotation.z = robot_pose.orientation.z
        t.transform.rotation.w = robot_pose.orientation.w
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)
        
        # Publish pose based on mode
        if self.use_2d_pose:
            # Create a Pose2D message
            pose_2d = Pose2D()
            pose_2d.x = robot_pose.position.x
            pose_2d.y = robot_pose.position.y
            pose_2d.theta = self.quaternion_to_yaw(robot_pose.orientation)
            
            # Publish the 2D pose
            self.publisher.publish(pose_2d)
        else:
            # Create a PoseStamped message with the full 3D pose
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.header.frame_id = 'world'  # Set appropriate frame
            pose_stamped.pose = robot_pose
            
            # Publish the 3D pose
            self.publisher.publish(pose_stamped)


def main(args=None):
    """
    Main function to initialize and run the ROS 2 node.
    """
    import argparse
    import sys
    
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Go1 Ground Truth Pose Publisher')
    parser.add_argument('--2d_pose', dest='use_2d_pose', type=str, default='false',
                        help='Publish 2D pose (x, y, theta) instead of 3D pose. Use "true" to enable.')
    
    # Parse known args to allow ROS args to pass through
    parsed_args, unknown = parser.parse_known_args()
    
    # Convert string to boolean
    use_2d_pose = parsed_args.use_2d_pose.lower() in ['true', '1', 'yes']
    
    # Initialize ROS 2 with remaining args
    rclpy.init(args=unknown)
    
    go1_gt_pose_publisher = Go1GTPosePublisher(use_2d_pose=use_2d_pose)
    
    try:
        rclpy.spin(go1_gt_pose_publisher)
    except KeyboardInterrupt:
        go1_gt_pose_publisher.get_logger().info('Node stopped by user')
    finally:
        go1_gt_pose_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()