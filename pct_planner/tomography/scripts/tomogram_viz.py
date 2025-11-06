#!/usr/bin/python3
"""
Tomogram visualization utilities.
Provides functions to convert tomogram data into point clouds for ROS visualization.
"""

import numpy as np


def generate_tomogram_pointcloud(
    layers_g: np.ndarray,
    layers_t: np.ndarray,
    VISPROTO_I: np.ndarray,
    VISPROTO_P: np.ndarray,
    center: np.ndarray,
    slice_dh: float
) -> np.ndarray:
    """
    Generate a point cloud from tomogram layers for visualization.

    This function creates a PointCloud representation of the tomogram by:
    1. Filtering overlapping layers to reduce visual clutter
    2. Propagating traversability costs between layers
    3. Creating XYZI points (X, Y, Z position + Intensity for traversability)

    Args:
        layers_g: Geometry layers (n_slice, dim_x, dim_y) - height of each grid cell per layer
        layers_t: Traversability layers (n_slice, dim_x, dim_y) - traversability cost per cell
        VISPROTO_I: Grid indices prototype (n_points, 2) - index mapping for visualization
        VISPROTO_P: Grid points prototype (n_points, 4) - XYZI template for grid points
        center: Map center coordinates (2,) - [center_x, center_y]
        slice_dh: Height difference between slices (float)

    Returns:
        global_points: Point cloud as numpy array (n_points, 4) with XYZI format
            where intensity represents traversability cost
    """
    n_slice = layers_g.shape[0]

    # Create copies for visualization to avoid modifying original data
    vis_g = layers_g.copy()
    vis_t = layers_t.copy()

    # Initialize layer points template and offset by map center
    layer_points = VISPROTO_P.copy()
    layer_points[:, :2] += center

    # Build the complete tomogram point cloud layer by layer
    global_points = None

    for i in range(n_slice - 1):
        # Filter overlapping layers to avoid visual clutter:
        # If the height difference between consecutive layers is less than slice_dh,
        # mark the lower layer as invalid (NaN) to hide it from visualization
        mask_h = (vis_g[i + 1] - vis_g[i]) < slice_dh
        vis_g[i, mask_h] = np.nan

        # Propagate minimum traversability cost to upper layer for overlapping regions
        vis_t[i + 1, mask_h] = np.minimum(vis_t[i, mask_h], vis_t[i + 1, mask_h])

        # Update layer points with current layer's geometry and traversability
        layer_points[:, 2] = vis_g[i, VISPROTO_I[:, 0], VISPROTO_I[:, 1]]  # Z position
        layer_points[:, 3] = vis_t[i, VISPROTO_I[:, 0], VISPROTO_I[:, 1]]  # Intensity (traversability)

        # Filter out invalid points (points with NaN values)
        valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]

        # Accumulate valid points into global point cloud
        if global_points is None:
            global_points = valid_points.copy()
        else:
            global_points = np.concatenate((global_points, valid_points), axis=0)

    # Add the last layer (always include it, no filtering needed)
    layer_points[:, 2] = vis_g[-1, VISPROTO_I[:, 0], VISPROTO_I[:, 1]]
    layer_points[:, 3] = vis_t[-1, VISPROTO_I[:, 0], VISPROTO_I[:, 1]]
    valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]

    if global_points is None:
        global_points = valid_points.copy()
    else:
        global_points = np.concatenate((global_points, valid_points), axis=0)

    return global_points
