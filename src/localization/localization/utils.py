#!/usr/bin/env python3
'''
"""
Utils like transforming pose and transform to 4x4 transformation matrix, converting laser scan to point cloud, ICP registration, etc.
"""

import tf_transformations
from sensor_msgs.msg import LaserScan
import numpy as np

def pose_to_matrix(pose):
    """Convert pose [x, y, z, qx, qy, qz, qw] to 4x4 transformation matrix."""
    T = np.eye(4)
    T[:3, 3] = pose[:3]  # translation
    quat = pose[3:]  # quaternion [qx, qy, qz, qw]
    T[:3, :3] = tf_transformations.quaternion_matrix(quat)[:3, :3]
    return T

def transform_to_matrix(transform):
    """Convert geometry_msgs Transform to 4x4 transformation matrix."""
    T = np.eye(4)
    T[0, 3] = transform.translation.x
    T[1, 3] = transform.translation.y
    T[2, 3] = transform.translation.z
    quat = [transform.rotation.x, transform.rotation.y, 
            transform.rotation.z, transform.rotation.w]
    T[:3, :3] = tf_transformations.quaternion_matrix(quat)[:3, :3]
    return T

def scan_to_pcd(msg: LaserScan):
    """
    Process ROS2 scan message to range and angle data.
    """
    # Get ranges and angles
    ranges = np.array(msg.ranges)
    angles = np.arange(
        msg.angle_min, 
        msg.angle_max + msg.angle_increment/2, 
        msg.angle_increment
        )

    # Ensure angles and ranges have the same number of elements
    if len(angles) > len(ranges):
        angles = angles[:len(ranges)]
    elif len(ranges) > len(angles):
        ranges = ranges[:len(angles)]

    # Filter out invalid range values (inf, nan, and out-of-bounds)
    valid_indices = np.where(
        (ranges > msg.range_min) & 
        (ranges < msg.range_max) & 
        np.isfinite(ranges)
    )[0]

    ranges = ranges[valid_indices]
    angles = angles[valid_indices]
    
    # Convert ranges and angles to 2d point cloud
    pcd = np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))
    return pcd

def icp_2d(previous_pcd, current_pcd, max_iterations, tolerance, distance_threshold):
    """
    Iterative Closest Point (ICP) algorithm for aligning 2D point clouds.

    Args:
        current_pcd: (N, 2) numpy array of 2D points representing the previous point cloud
        previous_pcd: (M, 2) numpy array of 2D points representing the current point cloud
        max_iterations: Maximum number of iterations to run ICP
        tolerance: tolerance to check for convergence
        distance_threshold: If provided, only consider point pairs within this distance
    """
    # Step 1: Initialize
    R, t = _initialize_alignment(current_pcd, previous_pcd)
    E_prev = np.inf
    errors = []
    
    for iteration in range(max_iterations):
        # Step 2: Apply current transformation
        aligned_current_pcd = _apply_alignment(current_pcd, R, t)
        
        # Step 3: Find correspondences
        pairs, distances = _return_closest_pairs(aligned_current_pcd, previous_pcd)
        
        # Step 3.5: Reject correspondences if threshold provided (Part c)
        if distance_threshold is not None:
            valid_mask = distances <= distance_threshold
            if np.sum(valid_mask) < 3:  # Need at least 3 points for alignment
                print(f"Warning: Only {np.sum(valid_mask)} valid correspondences. Stopping.")
                break
            pairs = pairs[valid_mask]
            distances = distances[valid_mask]
        
        # Step 4: Update alignment (get incremental transformation)
        R_new, t_new = _update_alignment(aligned_current_pcd[pairs[:, 0]], previous_pcd[pairs[:, 1]])
        
        # Step 5: Compose transformations
        # Total transformation: first apply (R, t), then apply (R_new, t_new)
        # So: R_total = R_new @ R, t_total = R_new @ t + t_new
        R = R_new @ R
        t = R_new @ t + t_new
        
        # Step 6: Compute error (MSE)
        E = np.mean(distances ** 2)
        errors.append(E)
        
        # Step 7: Check convergence
        if abs(E_prev - E) < tolerance:
            break
        
        E_prev = E
    
    # Convert R (2x2) and t (2,) to SE(2) transformation matrix (3x3)
    T = np.eye(3)
    T[:2, :2] = R
    T[:2, 2] = t
    
    return T
        

def _initialize_alignment(current_pcd, previous_pcd):
    """Return the initial rotation and translation."""
    # Initialize with identity rotation
    R = np.eye(2)
    
    # Initialize translation by aligning centroids
    # This provides a much better starting point than t=0
    current_pcd_centroid = np.mean(current_pcd, axis=0)
    previous_pcd_centroid = np.mean(previous_pcd, axis=0)
    t = previous_pcd_centroid - current_pcd_centroid
    
    return R, t        

def _apply_alignment(current_pcd, R, t):
    """Apply the current alignment and return the aligned current_pcd points."""
    # Apply transformation: aligned = R @ current_pcd^T + t
    # current_pcd is (N, 2), R is (2, 2), t is (2,)
    # Result: (R @ current_pcd.T).T + t = current_pcd @ R.T + t
    return (R @ current_pcd.T).T + t

def _return_closest_pairs(current_pcd, previous_pcd):
    """Return the closest pairs of points between current_pcd and previous_pcd.
    
    For each point in current_pcd, find the closest point in previous_pcd.
    
    Args:
        current_pcd: (N, 2) numpy array of 2D points
        previous_pcd: (M, 2) numpy array of 2D points
    
    Returns:
        pairs: (N, 2) numpy array where pairs[i] = [current_pcd_idx, previous_pcd_idx]
        distances: (N,) numpy array of distances to closest points
    """
    # Compute pairwise distances using broadcasting
    # current_pcd[:, None, :] has shape (N, 1, 2)
    # previous_pcd[None, :, :] has shape (1, M, 2)
    # diff has shape (N, M, 2)
    diff = current_pcd[:, None, :] - previous_pcd[None, :, :]
    distances_matrix = np.linalg.norm(diff, axis=2)  # Shape: (N, M)
    
    # Find closest previous_pcd point for each current_pcd point
    closest_current_indices = np.argmin(distances_matrix, axis=1)  # Shape: (N,)
    distances = distances_matrix[np.arange(len(current_pcd)), closest_current_indices]  # Shape: (N,)
    
    # Create pairs: (current_pcd_idx, previous_pcd_idx)
    pairs = np.column_stack((np.arange(len(current_pcd)), closest_current_indices))
    
    return pairs, distances

def _update_alignment(current_pcd_paired, previous_pcd_paired):
    """Update the alignment and return the new rotation and translation.
    
    Args:
        current_pcd_paired: (K, 2) numpy array of paired points from previous point cloud
        previous_pcd_paired: (K, 2) numpy array of paired points from current point cloud
    """
    # Implement the SVD-based least squares solution
    # current_pcd_paired and previous_pcd_paired are already paired (same length)
    
    # Step 1: Compute centroids
    previous_centroid = np.mean(current_pcd_paired, axis=0)
    current_centroid = np.mean(previous_pcd_paired, axis=0)
    
    # Step 2: Center the points
    previous_centered = current_pcd_paired - previous_centroid
    current_centered = previous_pcd_paired - current_centroid
    
    # Step 3: Compute cross-covariance matrix H
    H = current_centered.T @ previous_centered
    
    # Step 4: Compute SVD
    U, S, Vt = np.linalg.svd(H)
    
    # Step 5: Compute rotation    
    # Handle reflection case: ensure det(R) = +1 for 2D (2x2 matrix)
    R = U @ np.diag([1, np.linalg.det(U @ Vt)]) @ Vt
    
    # Step 6: Compute translation
    t = current_centroid - R @ previous_centroid    
    return R, t

# 추가함!
def map_to_pcd(map_msg, threshold=50):
    """
    Convert OccupancyGrid map to 2D point cloud (numpy array).
    """
    width = map_msg.info.width
    height = map_msg.info.height
    resolution = map_msg.info.resolution
    origin_x = map_msg.info.origin.position.x
    origin_y = map_msg.info.origin.position.y
    
    data = np.array(map_msg.data).reshape((height, width))
    
    y_idxs, x_idxs = np.where(data >= threshold)
    
    x_coords = origin_x + (x_idxs + 0.5) * resolution
    y_coords = origin_y + (y_idxs + 0.5) * resolution
    
    return np.column_stack((x_coords, y_coords))
'''

#12/11
#!/usr/bin/env python3
"""
Utils for localization: 
- Transformation matrix conversions
- LiDAR scan processing
- Optimized ICP (KDTree)
"""

import tf_transformations
from sensor_msgs.msg import LaserScan
import numpy as np
from scipy.spatial import cKDTree  # [핵심] 고속 연산을 위한 라이브러리

def pose_to_matrix(pose):
    """Convert pose [x, y, z, qx, qy, qz, qw] to 4x4 transformation matrix."""
    T = np.eye(4)
    T[:3, 3] = pose[:3]
    quat = pose[3:]
    T[:3, :3] = tf_transformations.quaternion_matrix(quat)[:3, :3]
    return T

def transform_to_matrix(transform):
    """Convert geometry_msgs Transform to 4x4 transformation matrix."""
    T = np.eye(4)
    T[0, 3] = transform.translation.x
    T[1, 3] = transform.translation.y
    T[2, 3] = transform.translation.z
    quat = [transform.rotation.x, transform.rotation.y, 
            transform.rotation.z, transform.rotation.w]
    T[:3, :3] = tf_transformations.quaternion_matrix(quat)[:3, :3]
    return T

def scan_to_pcd(msg: LaserScan):
    """
    Process ROS2 scan message to range and angle data.
    Returns: (N, 2) numpy array
    """
    ranges = np.array(msg.ranges)
    angles = np.arange(
        msg.angle_min, 
        msg.angle_max + msg.angle_increment/2, 
        msg.angle_increment
    )

    # 길이 맞추기
    if len(angles) > len(ranges):
        angles = angles[:len(ranges)]
    elif len(ranges) > len(angles):
        ranges = ranges[:len(angles)]

    # 유효하지 않은 값(inf, nan) 제거
    valid_indices = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
    ranges = ranges[valid_indices]
    angles = angles[valid_indices]
    
    # Polar -> Cartesian 변환
    pcd = np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))
    return pcd

def map_to_pcd(map_msg, threshold=50):
    """
    Convert OccupancyGrid map to 2D point cloud.
    """
    width = map_msg.info.width
    height = map_msg.info.height
    resolution = map_msg.info.resolution
    origin_x = map_msg.info.origin.position.x
    origin_y = map_msg.info.origin.position.y
    
    data = np.array(map_msg.data).reshape((height, width))
    
    # 점유된 칸(장애물)만 추출
    y_idxs, x_idxs = np.where(data >= threshold)
    
    x_coords = origin_x + (x_idxs + 0.5) * resolution
    y_coords = origin_y + (y_idxs + 0.5) * resolution
    
    return np.column_stack((x_coords, y_coords))

def icp_2d(previous_pcd, current_pcd, max_iterations=5, tolerance=1e-3, distance_threshold=None):
    """
    [Optimized] Iterative Closest Point (ICP) using Scipy KDTree.
    Aligns current_pcd (Source) to previous_pcd (Target).
    """
    # 점이 너무 적으면 계산 불가 (Identity 반환)
    if len(previous_pcd) < 5 or len(current_pcd) < 5:
        return np.eye(3)

    R = np.eye(2)
    # 초기 위치: 중심점(Centroid) 차이로 빠른 시작
    t = np.mean(previous_pcd, axis=0) - np.mean(current_pcd, axis=0)
    
    # [고속화 핵심] 타겟 포인트 클라우드로 KDTree 생성 (루프 밖에서 1회 수행)
    tree = cKDTree(previous_pcd)
    
    prev_error = np.inf

    for i in range(max_iterations):
        # 1. 변환 적용 (Apply Transform)
        current_aligned = (current_pcd @ R.T) + t
        
        # 2. 가장 가까운 점 검색 (Nearest Neighbor Search via KDTree)
        distances, indices = tree.query(current_aligned, k=1)
        
        # 3. 거리 임계값 적용 (Thresholding)
        if distance_threshold is not None:
            valid_mask = distances < distance_threshold
            # 유효한 매칭 점이 너무 적으면 중단
            if np.sum(valid_mask) < 3:
                break
            source_points = current_aligned[valid_mask]
            target_points = previous_pcd[indices[valid_mask]]
        else:
            source_points = current_aligned
            target_points = previous_pcd[indices]

        # 매칭된 점이 부족하면 중단
        if len(source_points) < 3:
            break

        # 4. SVD를 이용한 최적 변환 계산 (Least Squares)
        mu_s = np.mean(source_points, axis=0)
        mu_t = np.mean(target_points, axis=0)
        
        H = (source_points - mu_s).T @ (target_points - mu_t)
        U, _, Vt = np.linalg.svd(H)
        
        dR = Vt.T @ U.T
        
        # 반사(Reflection) 행렬 보정 (Det가 -1이면 뒤집힘 방지)
        if np.linalg.det(dR) < 0:
            Vt[1, :] *= -1
            dR = Vt.T @ U.T
            
        dt = mu_t - dR @ mu_s

        # 5. 변환 행렬 누적 업데이트
        R = dR @ R
        t = dR @ t + dt
        
        # 6. 수렴 여부 확인 (Error check)
        current_error = np.mean(distances**2)
        if abs(prev_error - current_error) < tolerance:
            break
        prev_error = current_error
    
    # 3x3 변환 행렬 구성 (SE2)
    T = np.eye(3)
    T[:2, :2] = R
    T[:2, 2] = t
    
    return T
