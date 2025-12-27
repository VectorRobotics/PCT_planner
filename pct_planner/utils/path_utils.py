#!/usr/bin/env python3

import numpy as np
from typing import Tuple, Optional
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PointStamped


def get_waypoint_from_traj(
    path_msg: Path,
    odom_msg: Odometry,
    lookahead_dist: float = 1.0
) -> Optional[PointStamped]:
    """
    Extract the next waypoint from a path using fixed lookahead distance.
    Uses 3D distance to respect height/floor differences.

    Args:
        path_msg: Path message containing the trajectory
        odom_msg: Current odometry (position and orientation)
        lookahead_dist: Lookahead distance in meters

    Returns:
        PointStamped message for the waypoint, or None if path is empty
    """
    if path_msg is None or len(path_msg.poses) == 0:
        return None

    # Extract current pose (3D)
    current_pose = np.array([
        odom_msg.pose.pose.position.x,
        odom_msg.pose.pose.position.y,
        odom_msg.pose.pose.position.z
    ])

    # Convert path to numpy array (Nx3)
    path_3d = np.array([
        [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
        for pose in path_msg.poses
    ])

    # Find closest point on path using 3D distance
    closest_idx = _find_closest_point_on_path_3d(current_pose, path_3d)

    # Find lookahead point index
    waypoint_idx = _find_lookahead_index(path_3d, closest_idx, lookahead_dist)

    # Get the waypoint from path
    waypoint = path_3d[waypoint_idx]

    # Create PointStamped message
    waypoint_msg = PointStamped()
    waypoint_msg.header = path_msg.header
    waypoint_msg.point.x = waypoint[0]
    waypoint_msg.point.y = waypoint[1]
    waypoint_msg.point.z = waypoint[2]

    return waypoint_msg


def _find_closest_point_on_path_3d(
    pose: np.ndarray,
    path: np.ndarray
) -> int:
    """
    Find the closest point on the path to current pose using 3D distance.
    Height differences are weighted more heavily to stay on correct floor.

    Args:
        pose: Current position [x, y, z]
        path: Path waypoints as Nx3 array

    Returns:
        Index of closest point
    """
    # Weight height difference more to prefer same-floor points
    diff = path - pose
    # Scale z difference by 3x to heavily penalize floor mismatches
    diff[:, 2] *= 3.0
    distances = np.linalg.norm(diff, axis=1)
    return np.argmin(distances)


def _find_lookahead_index(
    path: np.ndarray,
    start_idx: int,
    lookahead_dist: float
) -> int:
    """
    Find look-ahead point index on path at specified distance.

    Args:
        path: Path waypoints as Nx3 array
        start_idx: Starting index for search
        lookahead_dist: Lookahead distance in meters (2D, for driving)

    Returns:
        Index of look-ahead point
    """
    accumulated_dist = 0.0

    for i in range(start_idx, len(path) - 1):
        # Use 2D distance for lookahead (horizontal travel distance)
        segment_dist = np.linalg.norm(path[i + 1, :2] - path[i, :2])

        if accumulated_dist + segment_dist >= lookahead_dist:
            return i + 1

        accumulated_dist += segment_dist

    # If lookahead distance exceeds path, return the last point
    return len(path) - 1
