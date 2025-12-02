#!/usr/bin/env python3
'''
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
'''






"""
ROS2 global localizer node (Custom Particle Filter / MCL Implementation)
- Implements Monte Carlo Localization from scratch (No Nav2).
- Optimized with NumPy vectorization.
- Safe fallback: Uses SciPy if available, otherwise pure NumPy.
- Publishes /go1_pose for grading.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import tf_transformations
import numpy as np

# utils.py에서 함수 가져오기 (같은 폴더에 있어야 함)
from utils import map_to_pcd, pose_to_matrix

# [Safety Strategy] Check if scipy is installed
try:
    from scipy.spatial import cKDTree
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

class ParticleFilterLocalizer(Node):
    def __init__(self):
        super().__init__('global_localizer')
        
        # 1. Parameters (Launch File에서 넘어온 값들)
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('num_particles', 80)  #200은 너무 많아서 줄임
        
        self.initial_x = self.get_parameter('x').value
        self.initial_y = self.get_parameter('y').value
        self.initial_yaw = self.get_parameter('yaw').value
        self.num_particles = self.get_parameter('num_particles').value

        # Warning if Scipy is missing (Safety Check)
        if not HAS_SCIPY:
            self.get_logger().warn("SciPy not found! Running in slower NumPy fallback mode.")
            # Fallback mode needs fewer particles to maintain rate
            if self.num_particles > 100:
                self.num_particles = 100
                self.get_logger().warn(f"Reduced particles to {self.num_particles} for performance.")

        # Noise Parameters (Motion Model) - 튜닝 포인트
        self.odom_noise = [0.05, 0.05, 0.02] # x, y, yaw std_dev
        
        # Sensor Model Parameters - 튜닝 포인트
        self.scan_step = 15  # 최적화: 레이저 빔 15개당 1개만 계산 (속도 향상)
        self.map_sigma = 0.2 # Likelihood sigma (meters)

        # 2. ROS Setup
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # [중요 수정] 시뮬레이션 시간(Sim Time) 동기화를 위해 clock 전달 -> TF_OLD_DATA 에러 해결
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # [추가] 과제 제출용 Pose Publisher (/go1_pose)
        self.pose_pub = self.create_publisher(PoseStamped, '/go1_pose', 10)

        # 3. State Variables
        self.map_pcd = None
        self.particles = None 
        self.weights = None   
        self.T_map_odom = np.eye(4)
        self.last_odom_pose = None
        self.processing = False

        # Initialize Particles
        self.initialize_particles()
        
        # Publish initial TF immediately
        self.broadcast_tf(self.get_clock().now().to_msg())

    def initialize_particles(self):
        """초기 위치 주변에 가우시안 분포로 파티클 생성"""
        self.particles = np.empty((self.num_particles, 3))
        self.particles[:, 0] = self.initial_x + np.random.normal(0, 0.2, self.num_particles)
        self.particles[:, 1] = self.initial_y + np.random.normal(0, 0.2, self.num_particles)
        self.particles[:, 2] = self.initial_yaw + np.random.normal(0, 0.1, self.num_particles)
        self.weights = np.ones(self.num_particles) / self.num_particles
        self.get_logger().info(f"Initialized {self.num_particles} particles at ({self.initial_x}, {self.initial_y})")

    def map_callback(self, msg):
        if self.map_pcd is None:
            self.get_logger().info("Processing Map...")
            # utils.py의 map_to_pcd 사용
            self.map_pcd = map_to_pcd(msg, threshold=50)
            self.get_logger().info(f"Map Loaded. {len(self.map_pcd)} points.")

    def get_odom_pose(self):
        """odom -> base_link TF 가져오기"""
        try:
            # 타임아웃을 줘서 Extrapolation 에러 방지
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            q = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
            _, _, yaw = tf_transformations.euler_from_quaternion(q)
            return np.array([trans.transform.translation.x, trans.transform.translation.y, yaw])
        except Exception as e:
            # TF가 아직 안 들어오면 조용히 리턴
            return None

    def motion_update(self, current_odom_pose):
        """Step 1: Motion Update (Prediction)"""
        if self.last_odom_pose is None:
            return

        # Odom 기준 이동량 계산
        dx = current_odom_pose[0] - self.last_odom_pose[0]
        dy = current_odom_pose[1] - self.last_odom_pose[1]
        dyaw = current_odom_pose[2] - self.last_odom_pose[2]
        
        # Angle Wrapping
        dyaw = (dyaw + np.pi) % (2 * np.pi) - np.pi

        # 로봇 로컬 좌표계로 변환
        cos_yaw = np.cos(self.last_odom_pose[2])
        sin_yaw = np.sin(self.last_odom_pose[2])
        
        local_dx = cos_yaw * dx + sin_yaw * dy
        local_dy = -sin_yaw * dx + cos_yaw * dy

        # 파티클 이동 (Vectorized)
        p_cos = np.cos(self.particles[:, 2])
        p_sin = np.sin(self.particles[:, 2])

        # 노이즈 추가
        noise_x = np.random.normal(0, self.odom_noise[0], self.num_particles)
        noise_y = np.random.normal(0, self.odom_noise[1], self.num_particles)
        noise_yaw = np.random.normal(0, self.odom_noise[2], self.num_particles)

        self.particles[:, 0] += (p_cos * local_dx - p_sin * local_dy) + noise_x
        self.particles[:, 1] += (p_sin * local_dx + p_cos * local_dy) + noise_y
        self.particles[:, 2] += dyaw + noise_yaw
        
        self.particles[:, 2] = (self.particles[:, 2] + np.pi) % (2 * np.pi) - np.pi

    def measurement_update(self, scan_msg):
        """Step 2: Measurement Update (Correction)"""
        if self.map_pcd is None:
            return

        # 1. Scan Downsampling
        ranges = np.array(scan_msg.ranges)
        # 유효 범위 필터링
        valid_idxs = np.where((ranges > scan_msg.range_min) & (ranges < scan_msg.range_max))[0]
        # 속도 향상을 위해 띄엄띄엄 샘플링
        valid_idxs = valid_idxs[::self.scan_step]
        
        sampled_ranges = ranges[valid_idxs]
        angles = scan_msg.angle_min + valid_idxs * scan_msg.angle_increment
        
        # 로봇 기준 스캔 좌표
        scan_x = sampled_ranges * np.cos(angles)
        scan_y = sampled_ranges * np.sin(angles)

        # 2. Transform to World for all particles (Broadcasting)
        p_cos = np.cos(self.particles[:, 2])[:, np.newaxis]
        p_sin = np.sin(self.particles[:, 2])[:, np.newaxis]
        p_x = self.particles[:, 0][:, np.newaxis]
        p_y = self.particles[:, 1][:, np.newaxis]

        # 파티클 위치 기준으로 스캔 점들을 월드 좌표로 변환
        world_scan_x = p_x + scan_x * p_cos - scan_y * p_sin
        world_scan_y = p_y + scan_x * p_sin + scan_y * p_cos
        
        # 거리 계산을 위해 펼침 (N_particles * N_scan_points, 2)
        flat_scan_points = np.column_stack((world_scan_x.flatten(), world_scan_y.flatten()))

        # 3. Find Nearest Neighbor Distance
        # 최적화: 파티클 평균 위치 주변 맵만 잘라서 비교 (Local Map)
        mean_x = np.mean(self.particles[:, 0])
        mean_y = np.mean(self.particles[:, 1])
        
        dist_from_mean = np.linalg.norm(self.map_pcd - np.array([mean_x, mean_y]), axis=1)
        local_map = self.map_pcd[dist_from_mean < 4.0] # 반경 4m 맵만 사용

        if len(local_map) == 0:
            return

        dists = None

        # [SAFEGUARD] Use Scipy if available, else NumPy
        if HAS_SCIPY:
            tree = cKDTree(local_map)
            dists, _ = tree.query(flat_scan_points, k=1)
        else:
            # NumPy Fallback (Chunked to prevent memory explosion)
            dists_list = []
            chunk_size = 500 
            for i in range(0, len(flat_scan_points), chunk_size):
                chunk = flat_scan_points[i:i+chunk_size]
                # (Chunk, 1, 2) - (1, Map, 2)
                diff = chunk[:, np.newaxis, :] - local_map[np.newaxis, :, :]
                # 가장 가까운 거리
                min_d = np.min(np.linalg.norm(diff, axis=2), axis=1)
                dists_list.append(min_d)
            dists = np.concatenate(dists_list)

        dists = dists.reshape(self.num_particles, -1)
        
        # 4. Weighting (Gaussian Likelihood)
        # 거리가 가까울수록 가중치 높음
        weights = np.exp(-(dists**2) / (2 * self.map_sigma**2))
        self.weights = np.mean(weights, axis=1)
        
        # Normalize weights
        if np.sum(self.weights) > 0:
            self.weights /= np.sum(self.weights)
        else:
            self.weights = np.ones(self.num_particles) / self.num_particles

    def resampling(self):
        """Step 3: Resampling (Low Variance or Roulette)"""
        n_eff = 1.0 / np.sum(self.weights**2)
        # 파티클 다양성이 부족할 때만 리샘플링
        if n_eff < self.num_particles / 2:
            indices = np.random.choice(
                self.num_particles, 
                size=self.num_particles, 
                p=self.weights
            )
            self.particles = self.particles[indices]
            self.weights = np.ones(self.num_particles) / self.num_particles

    def scan_callback(self, msg):
        if self.processing or self.map_pcd is None:
            return
        self.processing = True

        current_odom_pose = self.get_odom_pose()
        if current_odom_pose is None:
            self.processing = False
            return

        # 1. MCL Cycle (Motion -> Measurement -> Resample)
        if self.last_odom_pose is not None:
            # 조금이라도 움직였을 때만 연산 (부하 감소)
            moved_dist = np.linalg.norm(current_odom_pose[:2] - self.last_odom_pose[:2])
            moved_angle = abs(current_odom_pose[2] - self.last_odom_pose[2])
            
            if moved_dist > 0.02 or moved_angle > 0.02:
                self.motion_update(current_odom_pose)
                self.measurement_update(msg)
                self.resampling()

        self.last_odom_pose = current_odom_pose

        # 2. Calculate Mean Pose (Weighted Average)
        mean_x = np.average(self.particles[:, 0], weights=self.weights)
        mean_y = np.average(self.particles[:, 1], weights=self.weights)
        
        # Yaw는 벡터 평균 (Circular Mean)
        sin_avg = np.average(np.sin(self.particles[:, 2]), weights=self.weights)
        cos_avg = np.average(np.cos(self.particles[:, 2]), weights=self.weights)
        mean_yaw = np.arctan2(sin_avg, cos_avg)

        # 3. Calculate map -> odom TF
        # T_map_base = T_map_odom * T_odom_base
        # T_map_odom = T_map_base * inv(T_odom_base)
        
        q_mean = tf_transformations.quaternion_from_euler(0, 0, mean_yaw)
        T_map_base = pose_to_matrix([mean_x, mean_y, 0.0, q_mean[0], q_mean[1], q_mean[2], q_mean[3]])
        
        q_odom = tf_transformations.quaternion_from_euler(0, 0, current_odom_pose[2])
        T_odom_base = pose_to_matrix([current_odom_pose[0], current_odom_pose[1], 0.0, q_odom[0], q_odom[1], q_odom[2], q_odom[3]])
        
        self.T_map_odom = T_map_base @ np.linalg.inv(T_odom_base)

        # 4. Broadcast TF and Publish Pose
        self.broadcast_tf(msg.header.stamp)
        self.publish_pose(msg.header.stamp, mean_x, mean_y, mean_yaw)
        
        self.processing = False

    def broadcast_tf(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        
        t.transform.translation.x = self.T_map_odom[0, 3]
        t.transform.translation.y = self.T_map_odom[1, 3]
        t.transform.translation.z = 0.0
        
        q = tf_transformations.quaternion_from_matrix(self.T_map_odom)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)

    def publish_pose(self, stamp, x, y, yaw):
        """[추가] 과제 제출용 /go1_pose 토픽 발행"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'map'
        
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0
        
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ParticleFilterLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()