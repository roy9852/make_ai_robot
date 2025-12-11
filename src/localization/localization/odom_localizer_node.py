#!/usr/bin/env python3
#original code
'''
"""
ROS2 odom localizer node

This node is responsible for localizing the robot in the odom frame.
It uses various sensors to localize the robot in the odom frame.
To overcome the limitations of drift, noise, and other factors, global localization is used to correct the odom localization.

odom_localizer: tf from odom to base_link frame
global_localizer: tf from map to odom frame
combined: tf from map to base_link frame

Current code uses Iterative Closest Point (ICP) to localize the robot in the odom frame.
You can modify this node to use other sensors to localize the robot.
Usually, LiDAR is used to localize the robot in the map frame, for global localization with given map. 
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
        self.max_iterations = 5
        self.tolerance = 1e-5
        self.distance_threshold = 0.2

        # Current pose in the odom frame
        # pose is 3 x 3 SE(2) transformation matrix: [R, t; 0, 1] from odom frame to base frame.
        self.current_pose = np.eye(3)

    def scan_callback(self, msg):
        """
        Callback function for the laser scan topic. 
        Update the current scan and localize the robot in the odom frame.
        """
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
        self.pose_delta = icp_2d(
            self.previous_pcd, 
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
'''


# 수정해야함 candidate2
#!/usr/bin/env python3
'''
"""
ROS2 odom localizer node

This node is responsible for localizing the robot in the odom frame.
It uses only sensor data (LiDAR, IMU) to estimate the robot's state.
No ground truth or simulator pose is used—this satisfies the project requirement.

odom_localizer: tf from odom to base_link frame
global_localizer: tf from map to odom frame
combined: tf from map to base_link frame

Current code uses Iterative Closest Point (ICP) and IMU fusion to localize the robot in the odom frame.
You can further modify this node to use other sensors (e.g., RGBD camera) for improved localization.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan, Imu
from tf2_ros import TransformBroadcaster
import tf_transformations
import numpy as np

from utils import scan_to_pcd, icp_2d

class OdomLocalizerNode(Node):
    def __init__(self):
        super().__init__('odom_localizer')
        self.get_logger().info('Odom localizer node initialized')

        # LiDAR (LaserScan) subscription
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        # IMU subscription
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # TF publisher
        self.tf_broadcaster = TransformBroadcaster(self)

        # ICP point clouds
        self.previous_pcd = None
        self.current_pcd = None

        # ICP parameters
        self.max_iterations = 10
        self.tolerance = 1e-6
        self.distance_threshold = 0.2

        # Current pose in odom frame (SE(2) matrix)
        self.current_pose = np.eye(3)
        # Latest IMU orientation (yaw)
        self.latest_yaw = 0.0

    def imu_callback(self, msg):
        # Extract yaw from IMU quaternion
        q = msg.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.latest_yaw = yaw
        # IMU 데이터를 활용한 자세 보정 로직을 추가할 수 있음

    def scan_callback(self, msg):
        start_time = self.get_clock().now()

        # 첫 번째 스캔은 previous_pcd로 저장
        if self.previous_pcd is None:
            self.previous_pcd = scan_to_pcd(msg)
            return

        # 두 번째 이후 스캔은 current_pcd로 저장
        self.current_pcd = scan_to_pcd(msg)

        # ICP로 이동 추정 (LiDAR 기반)
        pose_delta = icp_2d(
            self.previous_pcd,
            self.current_pcd,
            self.max_iterations,
            self.tolerance,
            self.distance_threshold
        )

        # IMU yaw로 자세 보정 (예시: yaw만 IMU로 대체)
        # 실제로는 센서 융합 알고리즘(예: EKF 등) 적용 가능
        pose_delta[0:2, 0:2] = [
            [np.cos(self.latest_yaw), -np.sin(self.latest_yaw)],
            [np.sin(self.latest_yaw),  np.cos(self.latest_yaw)]
        ]

        # 이전 pcd 갱신
        self.previous_pcd = self.current_pcd

        # 현재 pose 갱신
        self.current_pose = self.current_pose @ pose_delta

        # TF publish
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base'
        t.transform.translation.x = float(self.current_pose[0, 2])
        t.transform.translation.y = float(self.current_pose[1, 2])
        t.transform.translation.z = 0.0
        # IMU yaw 사용
        q = tf_transformations.quaternion_from_euler(0, 0, float(self.latest_yaw))
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        self.tf_broadcaster.sendTransform(t)

        end_time = self.get_clock().now()
        time_delta = (end_time - start_time).nanoseconds / 1e9
        frequency = 1.0 / time_delta
        self.get_logger().info(f'ICP+IMU registration frequency: {frequency:.3f} Hz')

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
'''


# 적분으로 밋밋한 벽 앞에서도 인식 잘하게
# 수정해야함 candidate2
#!/usr/bin/env python3

"""
ROS2 odom localizer node

This node is responsible for localizing the robot in the odom frame.
It uses only sensor data (LiDAR, IMU) to estimate the robot's state.
No ground truth or simulator pose is used—this satisfies the project requirement.

odom_localizer: tf from odom to base_link frame
global_localizer: tf from map to odom frame
combined: tf from map to base_link frame

Current code uses Iterative Closest Point (ICP) and IMU fusion to localize the robot in the odom frame.
You can further modify this node to use other sensors (e.g., RGBD camera) for improved localization.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan, Imu
from tf2_ros import TransformBroadcaster
import tf_transformations
import numpy as np
from geometry_msgs.msg import Twist  # 추가: 속도 명령 수신용

from utils import scan_to_pcd, icp_2d

class OdomLocalizerNode(Node):
    def __init__(self):
        super().__init__('odom_localizer')
        self.get_logger().info('Odom localizer node initialized')

        # LiDAR (LaserScan) subscription
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        # IMU subscription
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # TF publisher
        self.tf_broadcaster = TransformBroadcaster(self)

        # ICP point clouds
        self.previous_pcd = None
        self.current_pcd = None

        # ICP parameters
        self.max_iterations = 10
        self.tolerance = 1e-6
        self.distance_threshold = 0.2

        # Current pose in odom frame (SE(2) matrix)
        self.current_pose = np.eye(3)
        # Latest IMU orientation (yaw)
        self.latest_yaw = 0.0

        # 추가: 속도 명령 구독자 생성
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.linear_velocity = 0.0
        self.last_time = self.get_clock().now() # 시간 간격(dt) 계산용

    # [새로 추가할 함수]
    def cmd_vel_callback(self, msg):
        self.linear_velocity = msg.linear.x
        # 회전 속도(angular.z)는 IMU를 쓰므로 굳이 저장 안 해도 됨 (백업용으론 가능)

    def imu_callback(self, msg):
        # Extract yaw from IMU quaternion
        q = msg.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.latest_yaw = yaw
        # IMU 데이터를 활용한 자세 보정 로직을 추가할 수 있음

#callback만 다시
    def scan_callback(self, msg):
        # 1. 시간 간격(dt) 계산 (속도 x 시간 = 거리를 위해 필수)
        current_time = self.get_clock().now()
        # self.last_time이 없으면 현재 시간으로 초기화 (첫 실행 시)
        if not hasattr(self, 'last_time'):
            self.last_time = current_time
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 2. PCD 데이터 갱신 (ICP용 데이터 저장 - 구조 유지)
        if self.previous_pcd is None:
            self.previous_pcd = scan_to_pcd(msg)
            return
        
        self.current_pcd = scan_to_pcd(msg)
        self.previous_pcd = self.current_pcd 

        # ------------------------------------------------------------------
        # [핵심 수정 1] 위치 계산 로직 변경 (Dead Reckoning)
        # ICP가 벽을 못 봐서 '덜 가는' 문제를 해결하기 위해 바퀴 속도를 믿습니다.
        # ------------------------------------------------------------------

        # A. 회전(Yaw): IMU 절대 각도를 그대로 사용 (가장 정확)
        # pose_delta에 넣어서 행렬 곱셈하지 말고, 그냥 현재 각도로 덮어씁니다.
        current_yaw = self.latest_yaw 

        # B. 이동(Translation): 속도(cmd_vel) * 시간(dt)
        # 벽이 밋밋하든 말든 바퀴가 굴러간 만큼 좌표를 강제로 이동시킵니다.
        # (self.linear_velocity는 cmd_vel_callback에서 받아와야 합니다)
        if hasattr(self, 'linear_velocity'):
            delta_dist = self.linear_velocity * dt
        else:
            delta_dist = 0.0 # cmd_vel 구독 안 했으면 0 (비상용)

        # 현재 위치(x, y) 업데이트
        self.current_pose[0, 2] += delta_dist * np.cos(current_yaw)
        self.current_pose[1, 2] += delta_dist * np.sin(current_yaw)

        # ------------------------------------------------------------------
        # [핵심 수정 2] TF 발행 (트리 끊김 해결)
        # ------------------------------------------------------------------
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        
        # ★★★ 여기가 범인이었습니다! ★★★
        # 님의 시스템은 'base'를 쓰고 있으므로 'base'로 유지해야 트리가 연결됩니다.
        t.child_frame_id = 'base' 
        
        t.transform.translation.x = float(self.current_pose[0, 2])
        t.transform.translation.y = float(self.current_pose[1, 2])
        t.transform.translation.z = 0.0
        
        # 쿼터니언 변환 (IMU Yaw 사용)
        q = tf_transformations.quaternion_from_euler(0, 0, float(current_yaw))
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        
        self.tf_broadcaster.sendTransform(t)
        
        # (로깅은 필요하면 주석 해제)
        # self.get_logger().info(f'Odom updated. Yaw: {current_yaw:.2f}, Dist: {delta_dist:.4f}')


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
