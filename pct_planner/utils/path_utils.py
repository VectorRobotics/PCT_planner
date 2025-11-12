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

    Args:
        path_msg: Path message containing the trajectory
        odom_msg: Current odometry (position and orientation)
        lookahead_dist: Lookahead distance in meters

    Returns:
        PointStamped message for the waypoint, or None if path is empty
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

    # Find lookahead point
    waypoint_2d = _find_lookahead_point(path, closest_idx, lookahead_dist)

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
