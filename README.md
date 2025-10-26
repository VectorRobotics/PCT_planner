# PCT Planner - ROS 2 Jazzy Integration

## Overview

This is an implementation of paper **Efficient Global Navigational Planning in 3-D Structures Based on Point Cloud Tomography** (accepted by TMECH).
It provides a highly efficient and extensible global navigation framework based on a tomographic understanding of the environment to navigate ground robots in multi-layer structures.

This version has been integrated into the **Mecanum Wheel Platform Autonomy Stack** for ROS 2 Jazzy with the following enhancements:
- **ROS 2 Jazzy compatibility** with full node and launch file integration
- **Real-time tomogram generation** from live point cloud streams
- **Adaptive lookahead waypoint following** with curvature-based scaling
- **Goal validation** using BFS on traversability cost maps
- **Seamless integration** with FAR Planner-based local navigation

**Original Demonstrations**: [pct_planner](https://byangw.github.io/projects/tmech2024/)

![demo](rsc/docs/demo.png)

## Citing

If you use PCT Planner, please cite the following paper:

[Efficient Global Navigational Planning in 3-D Structures Based on Point Cloud Tomography](https://ieeexplore.ieee.org/document/10531813)

```bibtex
@ARTICLE{yang2024efficient,
  author={Yang, Bowen and Cheng, Jie and Xue, Bohuan and Jiao, Jianhao and Liu, Ming},
  journal={IEEE/ASME Transactions on Mechatronics},
  title={Efficient Global Navigational Planning in 3-D Structures Based on Point Cloud Tomography},
  year={2024},
  volume={},
  number={},
  pages={1-12}
}
```

## Prerequisites

### Environment

- **Ubuntu 24.04** (recommended) or Ubuntu 22.04
- **ROS 2 Jazzy** with ros-desktop-full installation
- **CUDA >= 11.7** (for GPU-accelerated tomogram processing)

### Python Dependencies

- **Python >= 3.10** (Python 3.12 recommended for Ubuntu 24.04)
- [CuPy](https://docs.cupy.dev/en/stable/install.html) with CUDA >= 11.7
- Open3D
- NumPy
- rclpy (ROS 2 Python client library)

## Build & Install

### 1. Install Python Dependencies

```bash
# Install CuPy (replace cu117 with your CUDA version, e.g., cu118, cu121)
pip3 install cupy-cuda11x

# Install other dependencies
pip3 install open3d numpy
```

### 2. Build the Planner Module

Inside the package, there are two modules:
- **tomography/** - Point cloud tomography for 3D volumetric map reconstruction
- **planner/** - Path planning and trajectory optimization

Build the planner module C++ dependencies:

```bash
cd src/route_planner/PCT_planner/pct_planner/planner/
./build_thirdparty.sh
./build.sh
```

### 3. Build the ROS 2 Package

From the workspace root:

```bash
cd autonomy_stack_mecanum_wheel_platform
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select pct_planner
source install/setup.bash
```

## Integration with Autonomy Stack

The PCT Planner integrates seamlessly with the existing autonomy stack and can be used as a drop-in replacement for the FAR Planner.

### Launch with Real Robot

**Default (FAR Planner):**
```bash
./system_real_robot_with_route_planner.sh
```

**With PCT Planner:**
```bash
ros2 launch vehicle_simulator system_real_robot_with_route_planner.launch use_pct_planner:=true
```

### Launch with Simulation

**With PCT Planner:**
```bash
ros2 launch vehicle_simulator system_simulation_with_route_planner.launch use_pct_planner:=true
```

### Launch with Bagfile

**With PCT Planner:**
```bash
ros2 launch vehicle_simulator system_bagfile_with_route_planner.launch use_pct_planner:=true
```

## ROS 2 Topics

### Subscribed Topics

- `/explored_areas` (sensor_msgs/PointCloud2) - Live point cloud map from SLAM
- `/state_estimation` (nav_msgs/Odometry) - Robot odometry
- `/goal_pose` (geometry_msgs/PoseStamped) - Goal position

### Published Topics

- `/global_path` (nav_msgs/Path) - Planned 3D trajectory
- `/way_point` (geometry_msgs/PointStamped) - Next waypoint for local planner (with adaptive lookahead)
- `/tomogram` (sensor_msgs/PointCloud2) - 3D volumetric map visualization
- `/tomogram_debug_grid` (nav_msgs/OccupancyGrid) - 2D cost grid for goal validation debugging

## Configuration

Parameters can be configured in `config/pct_planner_params.yaml`:

### Tomography Parameters

- `resolution`: Horizontal grid resolution (default: 0.15m)
- `slice_dh`: Vertical slice thickness (default: 0.5m)
- `slope_max`: Maximum traversable slope in radians (default: 0.6 rad ≈ 34°)
- `step_max`: Maximum step height (default: 0.5m)
- `cost_barrier`: Cost threshold for impassable terrain (default: 80.0)

### Waypoint Following Parameters (Adaptive Lookahead)

- `lookahead_distance`: Maximum lookahead on straight paths (default: 2.0m)
- `min_lookahead_distance`: Minimum lookahead on tight curves (default: 0.5m)
- `curvature_adaptive`: Enable curvature-based lookahead scaling (default: true)
- `curvature_scale`: Sensitivity to path curvature (default: 1.0, range: 0.5-2.0)

## Features

### 1. Real-Time Tomogram Generation

The planner automatically builds and updates the 3D volumetric map from incoming point cloud data:

```python
# Tomogram is built asynchronously from /explored_areas topic
# No manual PCD file processing needed
```

### 2. Adaptive Lookahead Waypoint Following

Implements curvature-adaptive lookahead for smooth path following:
- **Straight paths**: Uses full `lookahead_distance` for smooth tracking
- **Curved paths**: Reduces to `min_lookahead_distance` based on local curvature
- **Curvature computation**: Uses Menger curvature formula over path segments

### 3. Goal Validation

Validates and adjusts goals to safe, traversable locations:
- Converts tomogram layer to occupancy grid
- Uses BFS to find nearest safe goal if placed on obstacle
- Publishes debug grid to `/tomogram_debug_grid` for visualization

### 4. Multi-Layer 3D Planning

Handles complex multi-story environments with stairs, ramps, and overhangs using point cloud tomography.

## Standalone Examples (Original Functionality)

Three example scenarios are provided: **"Spiral"**, **"Building"**, and **"Plaza"**.

### Tomogram Construction from PCD

```bash
# Unzip example PCD files
cd rsc/pcd/
unzip pcd_files.zip

# Generate tomogram
cd ../../tomography/scripts/
python3 tomography.py --scene Building
```

### Trajectory Generation

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/../../planner/lib/3rdparty/gtsam-4.1.1/install/lib
cd ../../planner/scripts/
python3 plan.py --scene Building
```

## Architecture

```
┌──────────────────┐
│   Point Cloud    │
│   (LiDAR/SLAM)   │
└────────┬─────────┘
         │ /explored_areas
    ┌────▼─────────────┐
    │  PCT Planner     │
    │  Node            │
    ├──────────────────┤
    │ - Tomogram Gen   │
    │ - Path Planning  │
    │ - Goal Validation│
    │ - Waypoint Pub   │
    └────┬─────────────┘
         │
         ├─► /global_path (visualization)
         ├─► /way_point (→ Local Planner)
         ├─► /tomogram (visualization)
         └─► /tomogram_debug_grid (debug)
```

## ROS 2 Jazzy Integration

This package has been integrated into the Mecanum Wheel Platform Autonomy Stack for ROS 2 Jazzy.

### Additional Features

- **Real-time tomogram generation** from live point cloud streams (`/explored_areas`)
- **Adaptive lookahead waypoint following** with curvature-based scaling
- **Goal validation** using BFS on traversability cost maps
- **Seamless integration** with local planner via `/way_point` topic

### ROS 2 Build Instructions

From the workspace root:

```bash
cd autonomy_stack_mecanum_wheel_platform
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select pct_planner
source install/setup.bash
```

### Launch with Autonomy Stack

**Real Robot:**
```bash
ros2 launch vehicle_simulator system_real_robot_with_route_planner.launch use_pct_planner:=true
```

**Simulation:**
```bash
ros2 launch vehicle_simulator system_simulation_with_route_planner.launch use_pct_planner:=true
```

**Bagfile:**
```bash
ros2 launch vehicle_simulator system_bagfile_with_route_planner.launch use_pct_planner:=true
```

### ROS 2 Topics

**Subscribed:**
- `/explored_areas` - Live point cloud map
- `/state_estimation` - Robot odometry
- `/goal_pose` - Goal position

**Published:**
- `/global_path` - Planned trajectory
- `/way_point` - Next waypoint (adaptive lookahead)
- `/tomogram` - 3D volumetric map
- `/tomogram_debug_grid` - 2D cost grid (debug)

### Configuration

See `config/pct_planner_params.yaml` for all parameters including adaptive lookahead settings.

## License

The source code is released under [GPLv2](http://www.gnu.org/licenses/) license.

For commercial use, please contact Bowen Yang [byangar@connect.ust.hk](mailto:byangar@connect.ust.hk).
