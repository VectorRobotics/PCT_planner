#!/usr/bin/env python3

import numpy as np
from typing import Tuple, Optional
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PointStamped


def get_waypoint_from_traj(
    path_msg: Path,
    odom_msg: Odometry,
    lookahead_dist: float = 2.0,
    min_lookahead_dist: float = 0.5,
    curvature_adaptive: bool = True,
    curvature_scale: float = 1.0
) -> Optional[PointStamped]:
    """
    Extract the next waypoint from a path using adaptive lookahead based on path curvature.

    This implements "adaptive lookahead" - a common technique in path following where the
    lookahead distance is reduced on curved paths for tighter tracking and increased on
    straight paths for smoother following.

    Args:
        path_msg: Path message containing the trajectory
        odom_msg: Current odometry (position and orientation)
        lookahead_dist: Maximum lookahead distance in meters (used on straight paths)
        min_lookahead_dist: Minimum lookahead distance in meters (used on tight curves)
        curvature_adaptive: Enable adaptive lookahead based on path curvature
        curvature_scale: Scaling factor for curvature sensitivity (higher = more aggressive reduction)
                         Typical range: 0.5-2.0

    Returns:
        PointStamped message for the waypoint, or None if path is empty

    Notes:
        - On straight paths: Uses full lookahead_dist
        - On curved paths: Reduces to min_lookahead_dist based on curvature
        - Curvature computed from local path segments ahead of robot
    """
    if path_msg is None or len(path_msg.poses) == 0:
        return None

    # Extract current pose
    current_pose = np.array([
        odom_msg.pose.pose.position.x,
        odom_msg.pose.pose.position.y
    ])

    # Convert path to numpy array (Nx2)
    path = np.array([
        [pose.pose.position.x, pose.pose.position.y]
        for pose in path_msg.poses
    ])

    # Find closest point on path
    closest_idx, _ = _find_closest_point_on_path(current_pose, path)

    # Compute adaptive lookahead distance based on path curvature
    if curvature_adaptive:
        curvature = _compute_path_curvature(path, closest_idx)
        adaptive_lookahead = _scale_lookahead_by_curvature(
            lookahead_dist,
            min_lookahead_dist,
            curvature,
            curvature_scale
        )
    else:
        adaptive_lookahead = lookahead_dist

    # Find lookahead point
    waypoint_2d = _find_lookahead_point(path, closest_idx, adaptive_lookahead)

    # Get Z coordinate from the corresponding path point
    # Find the path index that corresponds to the waypoint
    waypoint_idx = min(closest_idx + 1, len(path_msg.poses) - 1)
    waypoint_z = path_msg.poses[waypoint_idx].pose.position.z

    # Create PointStamped message
    waypoint_msg = PointStamped()
    waypoint_msg.header = path_msg.header
    waypoint_msg.point.x = waypoint_2d[0]
    waypoint_msg.point.y = waypoint_2d[1]
    waypoint_msg.point.z = waypoint_z

    return waypoint_msg


def _find_closest_point_on_path(
    pose: np.ndarray,
    path: np.ndarray
) -> Tuple[int, np.ndarray]:
    """
    Find the closest point on the path to current pose.

    Args:
        pose: Current position [x, y]
        path: Path waypoints as Nx2 array

    Returns:
        Tuple of (closest_index, closest_point)
    """
    distances = np.linalg.norm(path - pose, axis=1)
    closest_idx = np.argmin(distances)
    return closest_idx, path[closest_idx]


def _compute_path_curvature(
    path: np.ndarray,
    start_idx: int,
    lookahead_window: int = 10
) -> float:
    """
    Compute the curvature of the path ahead of the robot.

    Uses the Menger curvature formula with three consecutive points to estimate
    path curvature. Averages curvature over a window of path segments.

    Args:
        path: Path waypoints as Nx2 array
        start_idx: Starting index (current position on path)
        lookahead_window: Number of path points ahead to analyze

    Returns:
        Average curvature in 1/meters (0 = straight, higher = tighter curve)
    """
    end_idx = min(start_idx + lookahead_window, len(path) - 2)

    if end_idx - start_idx < 2:
        return 0.0  # Not enough points to compute curvature

    curvatures = []

    # Compute curvature using three consecutive points (Menger curvature)
    for i in range(start_idx, end_idx):
        p1 = path[i]
        p2 = path[i + 1]
        p3 = path[i + 2]

        # Side lengths of triangle formed by three points
        a = np.linalg.norm(p2 - p1)
        b = np.linalg.norm(p3 - p2)
        c = np.linalg.norm(p3 - p1)

        # Avoid division by zero for collinear points
        if a < 1e-6 or b < 1e-6 or c < 1e-6:
            continue

        # Area of triangle using Heron's formula
        s = (a + b + c) / 2.0
        area_sq = s * (s - a) * (s - b) * (s - c)

        if area_sq > 0:
            area = np.sqrt(area_sq)
            # Menger curvature: K = 4 * Area / (a * b * c)
            curvature = 4.0 * area / (a * b * c)
            curvatures.append(curvature)

    # Return average curvature, or 0 if no valid curvatures computed
    return np.mean(curvatures) if curvatures else 0.0


def _scale_lookahead_by_curvature(
    max_lookahead: float,
    min_lookahead: float,
    curvature: float,
    scale: float
) -> float:
    """
    Scale lookahead distance based on path curvature.

    Uses exponential decay to smoothly reduce lookahead on curved paths:
    lookahead = min + (max - min) * exp(-scale * curvature)

    Args:
        max_lookahead: Maximum lookahead distance (straight paths)
        min_lookahead: Minimum lookahead distance (tight curves)
        curvature: Path curvature in 1/meters
        scale: Sensitivity to curvature (higher = more aggressive reduction)

    Returns:
        Scaled lookahead distance in meters
    """
    # Exponential decay based on curvature
    # When curvature = 0 (straight): returns max_lookahead
    # When curvature is high: approaches min_lookahead
    scaled = min_lookahead + (max_lookahead - min_lookahead) * np.exp(-scale * curvature)

    # Clamp to valid range
    return np.clip(scaled, min_lookahead, max_lookahead)


def _find_lookahead_point(
    path: np.ndarray,
    start_idx: int,
    lookahead_dist: float
) -> np.ndarray:
    """
    Find look-ahead point on path at specified distance.

    Args:
        path: Path waypoints as Nx2 array
        start_idx: Starting index for search
        lookahead_dist: Lookahead distance in meters

    Returns:
        Look-ahead point [x, y]
    """
    accumulated_dist = 0.0

    for i in range(start_idx, len(path) - 1):
        segment_dist = np.linalg.norm(path[i + 1] - path[i])

        if accumulated_dist + segment_dist >= lookahead_dist:
            # Interpolate to find exact lookahead point
            remaining_dist = lookahead_dist - accumulated_dist
            t = remaining_dist / segment_dist
            carrot = path[i] + t * (path[i + 1] - path[i])
            return carrot

        accumulated_dist += segment_dist

    # If lookahead distance exceeds path, return the last point
    return path[-1]
