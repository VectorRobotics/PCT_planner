# PCT Planner Configuration Parameters

This document describes all configurable parameters for the PCT (Perceptual Coordinated Tomography) Planner and provides guidance for tuning them to fix issues like "planning through walls."

## Overview

The PCT Planner uses a tomographic representation of the environment to compute safe, traversable paths. Parameters control how the environment is discretized, how traversability is assessed, and how obstacles are treated.

## Configuration Files

- **YAML Config**: `config/pct_planner_params.yaml` - User-editable parameters
- **Launch File**: `launch/pct_planner.launch.py` - Loads parameters and starts the node
- **Node**: `scripts/pct_planner_node.py` - Declares and uses ROS 2 parameters

## Parameters Reference

### Map Resolution Parameters

#### `resolution` (float, meters)
- **Default**: 0.3 m
- **Description**: Horizontal grid cell size for tomogram discretization
- **Impact**:
  - **Lower** (0.1-0.2): Higher detail, better obstacle detection, more computation
  - **Higher** (0.4-0.6): Faster computation, may miss small obstacles
- **Tuning for "planning through walls"**:
  - **Decrease to 0.15-0.2** to capture wall details more accurately
  - Must balance with computational performance

#### `slice_dh` (float, meters)
- **Default**: 0.5 m
- **Description**: Vertical thickness of each tomogram layer
- **Impact**:
  - **Lower** (0.3-0.4): Better vertical resolution, more accurate height modeling
  - **Higher** (0.6-1.0): Faster computation, may miss height variations
- **Tuning**: Keep at 0.4-0.5 for most indoor environments

#### `ground_h` (float, meters)
- **Default**: 0.0 m
- **Description**: Ground height reference level (0.0 = auto-detect from point cloud)
- **Impact**: Sets the base height for tomogram slicing
- **Tuning**: Usually keep at 0.0 for automatic detection

### Traversability Analysis Parameters

#### `kernel_size` (int, grid cells)
- **Default**: 7
- **Description**: Size of neighborhood window for traversability assessment
- **Impact**:
  - **Smaller** (3-5): More sensitive to local obstacles, may be overly conservative
  - **Larger** (9-11): Smoother assessment, may miss small obstacles
- **Tuning for "planning through walls"**:
  - **Increase to 9-11** to ensure obstacles are properly detected in their neighborhood
- **Constraint**: Must be odd number (3, 5, 7, 9, 11, etc.)

#### `slope_max_deg` (float, degrees)
- **Default**: 30.0°
- **Description**: Maximum passable slope angle
- **Impact**:
  - **Lower** (15-20°): More conservative, rejects steep terrain
  - **Higher** (35-45°): Allows steeper slopes, may be unsafe
- **Tuning for "planning through walls"**:
  - **Decrease to 20-25°** to be more conservative about terrain
  - Walls often have high slope values and should be rejected

#### `step_max` (float, meters)
- **Default**: 0.3 m
- **Description**: Maximum vertical step/discontinuity the robot can traverse
- **Impact**:
  - **Lower** (0.1-0.2): More conservative, rejects small obstacles
  - **Higher** (0.4-0.6): Allows larger steps
- **Tuning for "planning through walls"**:
  - **Decrease to 0.15-0.2** to be more conservative about vertical obstacles
  - Walls may appear as large vertical discontinuities

#### `standable_ratio` (float, 0.0-1.0)
- **Default**: 0.5
- **Description**: Minimum proportion of "standable" cells required within the kernel
- **Impact**:
  - **Lower** (0.2-0.3): Allows planning through rougher terrain
  - **Higher** (0.6-0.8): Requires more stable/flat terrain
- **Tuning for "planning through walls"**:
  - **Increase to 0.6-0.7** to require more stable terrain
  - This helps reject areas near walls or obstacles

### Safety and Inflation Parameters

#### `safe_margin` (float, meters)
- **Default**: 0.3 m
- **Description**: Minimum clearance around obstacles (robot safety buffer)
- **Impact**:
  - **Lower** (0.1-0.2): Tight passages allowed, higher collision risk
  - **Higher** (0.5-0.8): Wider safety buffer, more conservative
- **Tuning for "planning through walls"**:
  - **Increase to 0.5-0.7** to keep robot farther from walls
  - Should be larger than robot's physical radius

#### `inflation` (float, meters)
- **Default**: 0.2 m
- **Description**: Additional inflation beyond safety margin for cost gradient
- **Impact**:
  - **Lower** (0.1): Sharper cost gradient, may plan close to obstacles
  - **Higher** (0.3-0.5): Gradual cost gradient, smoother paths away from obstacles
- **Tuning for "planning through walls"**:
  - **Increase to 0.4-0.5** to create larger cost gradient around walls
  - Total clearance = `safe_margin + inflation`

## Recommended Settings for Fixing "Planning Through Walls"

If the planner is currently planning through walls, try this conservative configuration:

```yaml
/**:
  ros__parameters:
    # Higher resolution for better wall detection
    resolution: 0.2
    slice_dh: 0.4
    ground_h: 0.0

    # More conservative traversability
    kernel_size: 9
    slope_max_deg: 20.0
    step_max: 0.20
    standable_ratio: 0.6

    # Larger safety margins
    safe_margin: 0.6
    inflation: 0.4
```

## Tuning Workflow

1. **Start with conservative settings** (see above)
2. **Test in simulation or controlled environment**
3. **Gradually relax constraints** if paths are too conservative:
   - Increase `slope_max_deg` by 5° increments
   - Decrease `standable_ratio` by 0.1 increments
   - Decrease `safe_margin` and `inflation` by 0.1m increments
4. **Monitor performance**:
   - Check if paths avoid obstacles properly
   - Verify computational performance is acceptable
   - Ensure paths are not overly conservative

## Advanced Tuning

### For Different Environments

**Indoor (buildings, corridors)**:
- Lower `resolution` (0.15-0.2) for wall detail
- Higher `safe_margin` (0.5-0.7) for tight spaces
- Higher `kernel_size` (9-11) for obstacle detection

**Outdoor (rough terrain)**:
- Higher `slope_max_deg` (30-35°) for hills
- Lower `step_max` (0.15-0.2) for safety
- Moderate `resolution` (0.3-0.4) for performance

**Multi-story buildings**:
- Lower `slice_dh` (0.3-0.4) for stairs/ramps
- Higher `kernel_size` (11) for level detection

### Performance Optimization

If planning is too slow:
1. Increase `resolution` (but not above 0.4)
2. Increase `slice_dh` (but not above 0.6)
3. Decrease `kernel_size` (but not below 5)

## Parameter Override via Launch File

You can override parameters when launching:

```bash
ros2 launch pct_planner pct_planner.launch.py params_file:=/path/to/custom_params.yaml
```

Or create environment-specific config files:
- `pct_planner_params_indoor.yaml`
- `pct_planner_params_outdoor.yaml`
- `pct_planner_params_conservative.yaml`

## Debugging Tips

1. **Visualize the tomogram**: Check `/tomogram` topic in RViz to see traversability costs
2. **Check parameter loading**: Look for parameter log messages when node starts
3. **Monitor planning results**: Compare planned paths with obstacle locations
4. **Adjust incrementally**: Change one parameter at a time and observe effects

## Additional Parameters (Future Enhancement)

The following parameters are currently hardcoded but could be made configurable:

- `interval_min`: Minimum vertical clearance (currently 0.1m in dynamic config)
- `interval_free`: Free space interval threshold (currently 0.2m)
- `cost_barrier`: Cost assigned to impassable terrain (currently 100.0)
- Gateway detection thresholds
- Point cloud minimum threshold

## References

- **Paper**: "Efficient Multi-Level Topological Path Planning for Large-Scale Environments" (Yang et al., 2024)
- **Code**: `pct_planner/scripts/pct_planner.py` - TomogramConfig dataclass
- **Traversability**: `pct_planner/tomography/config/scene.py` - SceneTrav class
