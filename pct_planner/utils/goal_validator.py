from typing import Optional, Tuple
from collections import deque
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose


def tomogram_to_occupancy_grid(
    layer_cost: np.ndarray,
    resolution: float,
    center: np.ndarray,
) -> OccupancyGrid:
    """
    Convert a single tomogram layer to ROS OccupancyGrid format.

    Args:
        layer_cost: (dim_x, dim_y) - traversability cost grid from tomogram
                    Tomogram stores as (n_slices, dim_x, dim_y) where:
                    - dim_x corresponds to X axis (rows)
                    - dim_y corresponds to Y axis (columns)
                    - Values: 0 = free, higher = more costly, NaN = unknown
        resolution: grid resolution in meters
        center: [cx, cy] center of the map in world frame

    Returns:
        OccupancyGrid message compatible with find_safe_goal_bfs
    """
    dim_x, dim_y = layer_cost.shape

    # Create OccupancyGrid message
    # OccupancyGrid convention: width is X dimension, height is Y dimension
    grid = OccupancyGrid()
    grid.info.resolution = resolution
    grid.info.width = dim_x  # X dimension
    grid.info.height = dim_y  # Y dimension

    # Calculate origin (bottom-left corner of the grid)
    # Tomogram is centered, so origin = center - (dim/2) * resolution
    grid.info.origin = Pose()
    grid.info.origin.position.x = center[0] - (dim_x / 2.0) * resolution
    grid.info.origin.position.y = center[1] - (dim_y / 2.0) * resolution
    grid.info.origin.position.z = 0.0
    grid.info.origin.orientation.w = 1.0

    # Convert cost data to OccupancyGrid format
    # OccupancyGrid stores data in row-major order: data[y * width + x]
    # This means Y varies fastest (iterate through X rows, for each X iterate through Y)
    grid_data = np.zeros(dim_y * dim_x, dtype=np.int8)

    for ix in range(dim_x):
        for iy in range(dim_y):
            cost_value = layer_cost[ix, iy]
            grid_idx = iy * dim_x + ix  # Row-major: y * width + x

            if np.isnan(cost_value):
                grid_data[grid_idx] = -1  # Unknown
            else:
                # Clamp to 0-100 range and convert to int8
                grid_data[grid_idx] = int(np.clip(cost_value, 0, 100))

    grid.data = grid_data.tolist()

    return grid


def select_layer_for_height(
    z: float,
    slice_h0: float,
    slice_dh: float,
    n_slices: int
) -> Optional[int]:
    """
    Determine which tomogram layer corresponds to a given height.

    Args:
        z: height in world frame
        slice_h0: height of the first slice
        slice_dh: height interval between slices
        n_slices: total number of slices

    Returns:
        layer index, or None if height is out of range
    """
    if z < slice_h0:
        return None

    layer_idx = int(round((z - slice_h0) / slice_dh))

    if layer_idx < 0 or layer_idx >= n_slices:
        return None

    return layer_idx


def find_safe_goal_bfs(
    costmap: OccupancyGrid,
    goal: Tuple[float, float],  # (x, y) in world coordinates
    cost_threshold: int,
    min_clearance: float,
    max_search_distance: float,
    connectivity_check_radius: int,
) -> Optional[Tuple[float, float]]:
    """
    BFS-based search for nearest safe goal position.
    This guarantees finding the closest valid position.

    Pros:
    - Guarantees finding the closest safe position
    - Can check connectivity to avoid isolated spots
    - Efficient for small to medium search areas

    Cons:
    - Can be slower for large search areas
    - Memory usage scales with search area
    """
    
    # Extract costmap parameters
    resolution = costmap.info.resolution
    width = costmap.info.width
    height = costmap.info.height
    origin_x = costmap.info.origin.position.x
    origin_y = costmap.info.origin.position.y
    
    # Reshape flat data array to 2D grid (row-major order)
    grid = np.array(costmap.data, dtype=np.int8).reshape((height, width))
    
    # Convert goal to grid coordinates (world_to_grid equivalent)
    gx = int((goal[0] - origin_x) / resolution)
    gy = int((goal[1] - origin_y) / resolution)
    
    # Convert distances to grid cells
    clearance_cells = int(np.ceil(min_clearance / resolution))
    max_search_cells = int(np.ceil(max_search_distance / resolution))
    
    # BFS queue and visited set
    queue = deque[tuple[int, int, int]]([(gx, gy, 0)])
    visited = set[tuple[int, int]]([(gx, gy)])
    
    # 8-connected neighbors
    neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    
    while queue:
        x, y, dist = queue.popleft()
        
        # Check if we've exceeded max search distance
        if dist > max_search_cells:
            break
        
        # Check if position is valid
        if _is_position_safe(
            grid, width, height, x, y, cost_threshold, clearance_cells, connectivity_check_radius
        ):
            # Convert back to world coordinates (grid_to_world equivalent)
            world_x = origin_x + x * resolution
            world_y = origin_y + y * resolution
            return (world_x, world_y)
        
        # Add neighbors to queue
        for dx, dy in neighbors:
            nx, ny = x + dx, y + dy
            
            # Check bounds
            if 0 <= nx < width and 0 <= ny < height:
                if (nx, ny) not in visited:
                    visited.add((nx, ny))
                    queue.append((nx, ny, dist + 1))
    
    return None


def _is_position_safe(
    grid: np.ndarray,
    width: int,
    height: int,
    x: int,
    y: int,
    cost_threshold: int,
    clearance_cells: int,
    connectivity_check_radius: int,
) -> bool:
    """
    Check if a position is safe based on multiple criteria.

    Args:
        grid: The occupancy grid data as 2D numpy array
        width: Grid width in cells
        height: Grid height in cells
        x, y: Grid coordinates to check
        cost_threshold: Maximum acceptable cost
        clearance_cells: Minimum clearance in cells
        connectivity_check_radius: Radius to check for connectivity

    Returns:
        True if position is safe, False otherwise
    """
    
    # Check bounds first
    if not (0 <= x < width and 0 <= y < height):
        return False
    
    # Check if position itself is free
    # In ROS OccupancyGrid: -1 = unknown, 0 = free, 100 = occupied
    if grid[y, x] >= cost_threshold or grid[y, x] == -1:
        return False
    
    # Check clearance around position
    for dy in range(-clearance_cells, clearance_cells + 1):
        for dx in range(-clearance_cells, clearance_cells + 1):
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height:
                # Check if within circular clearance
                if dx * dx + dy * dy <= clearance_cells * clearance_cells:
                    if grid[ny, nx] >= cost_threshold:
                        return False
    
    # Check connectivity (not surrounded by obstacles)
    # Count free neighbors in a larger radius
    free_count = 0
    total_count = 0
    
    for dy in range(-connectivity_check_radius, connectivity_check_radius + 1):
        for dx in range(-connectivity_check_radius, connectivity_check_radius + 1):
            if dx == 0 and dy == 0:
                continue
            
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height:
                total_count += 1
                if grid[ny, nx] < cost_threshold and grid[ny, nx] != -1:
                    free_count += 1
    
    # Require at least 50% of neighbors to be free (not surrounded)
    if total_count > 0 and free_count < total_count * 0.5:
        return False
    
    return True