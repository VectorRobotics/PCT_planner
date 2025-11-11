# PCT Planner Smoothing & Layer Switching Optimization Guide

## Summary of Changes

This guide documents the recent improvements to expose path smoothing parameters and add comprehensive debugging for the PCT planner's path generation pipeline.

### Changes Made

1. **Exposed Hardcoded Smoothing Parameters to YAML Config**
2. **Added Comprehensive Debug Output for A* and GPMP Optimization**
3. **Enhanced Layer Switching Diagnostics**
4. **Updated Python Interface to Support New Parameters**

---

## New Configuration Parameters

All new parameters have been added to `config/pct_planner_params.yaml`:

### Path Smoothing Parameters (GPMP Optimizer)

```yaml
# sample_interval: Subsampling interval for A* path before optimization
# Higher values = fewer waypoints = smoother but less accurate paths
# Lower values = more waypoints = more accurate but potentially less smooth
# Range: 3-20, recommended: 10
sample_interval: 10

# interpolate_num: Number of interpolated points between each waypoint pair
# Higher values = denser trajectory = smoother motion
# Lower values = sparser trajectory = faster computation
# Range: 4-16, recommended: 8
interpolate_num: 8

# max_iterations: Maximum optimization iterations for trajectory smoothing
# Higher values = better convergence but slower computation
# Lower values = faster but may not converge for complex paths
# Range: 50-300, recommended: 100-150
max_iterations: 100

# lambda_initial: Levenberg-Marquardt damping factor for optimization
# Higher values = more conservative/stiff optimization (slower to change path)
# Lower values = more aggressive optimization (may overshoot)
# Range: 10-500, recommended: 100-200
lambda_initial: 200.0

# Qc: Gaussian process noise for position smoothness (squared)
# Lower values = smoother paths (stronger smoothing constraint)
# Higher values = paths closer to A* output (weaker smoothing)
# Range: 0.001-0.1, recommended: 0.01
# Actual parameter is Qc = qc_position^2
qc_position: 0.1

# Qc_heading: Gaussian process noise for heading smoothness
# Lower values = smoother heading changes
# Higher values = allow sharper turns
# Range: 0.001-0.05, recommended: 0.01
qc_heading: 0.01
```

### Debug Flags

```yaml
# enable_path_debug: Print detailed A* pathfinding debug info
enable_path_debug: false

# enable_optimizer_debug: Print GPMP optimization debug info
enable_optimizer_debug: false
```

---

## Understanding the Debug Output

### A* Pathfinding Output

When `enable_path_debug: true`, you'll see:

```
[A*] Start searching: start=[layer,y,x], goal=[layer,y,x]
[A*] Start height=X.XX, goal height=X.XX
[A*] cost_threshold=XX.X, step_cost_weight=X.XX
[A*] Layer transition at iter N: L1->L2 (height=X.XX, cost=X.X)
[A*] Path found: N waypoints, M iterations, K layer transitions, X.XXms
[A*] Path layer changes: N total, max M consecutive
```

**Key Metrics:**
- **Layer transitions**: High numbers (>20% of waypoints) indicate excessive layer switching
- **Max consecutive changes**: >3 suggests zigzagging between layers
- **Iterations**: Higher means more complex pathfinding (normal range: 100-5000)

### GPMP Optimizer Output

When `enable_optimizer_debug: true`, you'll see:

```
[GPMP] === Trajectory Optimization Parameters ===
[GPMP] sample_interval: 10
[GPMP] interpolate_num: 8
[GPMP] max_iterations: 100
[GPMP] lambda_initial: 200.0
[GPMP] qc_position: 0.1000 (kQc=0.010000)
[GPMP] qc_heading: 0.0100
[GPMP] Input path size: N, Subsampled path size: M
[GPMP] dt: X.XXXX, tau: X.XXXX
[GPMP] Optimization finished: N=M, iterations=K/100, time=XX.XXms, cost=X.XXXX
```

**Warning Indicators:**
- `[WARNING: Max iterations reached, may not have converged!]`
  - Solution: Increase `max_iterations` or adjust `lambda_initial`
- `[WARNING: High cost, poor convergence!]` (cost > 50)
  - Solution: Lower `qc_position` for stronger smoothing, or check A* path quality

---

## Troubleshooting Zigzag/Discontinuous Paths

### Diagnostic Steps

1. **Enable Debug Mode**
   ```yaml
   enable_path_debug: true
   enable_optimizer_debug: true
   ```

2. **Run Your Planning Task** and observe the output

3. **Analyze A* Output**
   - Look for frequent layer transitions
   - Check if `max consecutive changes` > 3
   - High consecutive changes = layer switching issue

4. **Analyze GPMP Output**
   - Check if optimizer reaches max iterations
   - Check if cost > 50 (poor convergence)
   - High cost = smoothing struggling to resolve A* zigzags

### Common Issues & Solutions

#### Issue 1: Excessive Layer Transitions in A*

**Symptoms:**
- `[A*] Path layer changes: 50 total, max 8 consecutive`
- Zigzag pattern in raw A* path

**Root Cause:**
- DecideLayer() function switching layers too aggressively
- Ambiguous elevation data causing rapid layer changes

**Solutions:**
1. Review your tomogram generation - ensure layers are well-separated
2. Increase `astar_cost_threshold` to make A* more conservative:
   ```yaml
   astar_cost_threshold: 80.0  # Increase from 50
   ```
3. The DecideLayer logic has tight (0.2m) and relaxed (0.6m) height constraints - these are hardcoded

#### Issue 2: Optimizer Not Converging

**Symptoms:**
- `[WARNING: Max iterations reached, may not have converged!]`
- Jerky, discontinuous output trajectory

**Solutions:**

**Option A: Increase Iterations**
```yaml
max_iterations: 200  # Increase from 100
```

**Option B: Reduce Damping (More Aggressive)**
```yaml
lambda_initial: 100.0  # Reduce from 200.0
```

**Option C: Stronger Smoothing**
```yaml
qc_position: 0.07  # Reduce from 0.1 (stronger smoothing)
```

#### Issue 3: Over-Smoothed Path (Too Conservative)

**Symptoms:**
- Path cuts corners or deviates from optimal route
- Cost < 5 but path doesn't follow A* closely

**Solutions:**

**Weaken Smoothing:**
```yaml
qc_position: 0.15  # Increase from 0.1 (weaker smoothing)
```

**Reduce Subsampling:**
```yaml
sample_interval: 5  # Reduce from 10 (preserve more A* waypoints)
```

#### Issue 4: Sparse Trajectory (Not Smooth Enough)

**Symptoms:**
- Robot has jerky motion following path
- Large gaps between trajectory points

**Solutions:**

**Increase Interpolation:**
```yaml
interpolate_num: 12  # Increase from 8
```

**Reduce Subsampling:**
```yaml
sample_interval: 7  # Reduce from 10
```

---

## Recommended Tuning Workflow

### Step 1: Baseline Test
```yaml
enable_path_debug: true
enable_optimizer_debug: true
sample_interval: 10
interpolate_num: 8
max_iterations: 100
lambda_initial: 200.0
qc_position: 0.1
qc_heading: 0.01
```

Run your planning task and collect metrics.

### Step 2: Address A* Issues First

If A* shows excessive layer transitions:
1. Check your tomogram data quality
2. Increase `astar_cost_threshold`
3. Consider adjusting `safe_cost_margin`

### Step 3: Tune Optimizer for Smoothness

For smoother paths:
```yaml
qc_position: 0.07      # Stronger smoothing
max_iterations: 150    # More time to converge
sample_interval: 8     # Preserve more detail
```

For more responsive paths:
```yaml
qc_position: 0.12      # Weaker smoothing
lambda_initial: 150.0  # More aggressive
interpolate_num: 10    # Denser output
```

### Step 4: Fine-Tune Convergence

Monitor convergence cost:
- **Cost < 10**: Excellent convergence
- **Cost 10-30**: Good convergence
- **Cost 30-50**: Acceptable, may benefit from tuning
- **Cost > 50**: Poor convergence, needs intervention

---

## Code Changes Reference

### Files Modified

1. **config/pct_planner_params.yaml**
   - Added smoothing parameters
   - Added debug flags

2. **planner/lib/src/trajectory_optimization/gpmp_optimizer/gpmp_optimizer.h**
   - Added parameter setters
   - Added member variables for configurable parameters

3. **planner/lib/src/trajectory_optimization/gpmp_optimizer/gpmp_optimizer.cc**
   - Removed hardcoded kQc and kQcHeading constants
   - Compute from configurable parameters
   - Added comprehensive debug output
   - Added convergence warnings

4. **planner/lib/src/a_star/a_star_search.h**
   - Added Debug(bool enable) overload

5. **planner/lib/src/a_star/a_star_search.cc**
   - Enhanced search output with layer transition tracking
   - Added path characteristic analysis
   - Improved failure diagnostics

6. **planner/lib/src/ele_planner/offline_ele_planner.h**
   - Added parameter setters that forward to optimizers
   - Added set_path_debug() method

7. **planner/lib/src/ele_planner/python_interface.cc**
   - Exposed new setter methods to Python

8. **planner/scripts/planner_wrapper.py**
   - Load parameters from config
   - Configure planner with parameters
   - Configure debug flags

### Building

```bash
cd autonomy_stack_mecanum_wheel_platform
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select pct_planner
source install/setup.bash
```

---

## Advanced: Layer Switching Logic Analysis

The `DecideLayer()` function in A* has a multi-stage decision process:

### Stage 1: Direct Elevation Check
- If current node has `ele > 0.5`: Try layer+1 (upward stairs)
- If current node has `ele < -0.5`: Try layer-1 (downward stairs)
- Height difference must be ≤ 0.5m

### Stage 2: Tight Constraint Search
- Search nearby layers (±1, ±2, ...)
- Height difference must be ≤ 0.2m (tight)
- Look for elevation indicators in nearby layers

### Stage 3: Relaxed Constraint (High Cost Fallback)
- Only triggered if current layer cost ≥ `cost_threshold`
- Height difference allowed up to 0.6m (relaxed)
- Prefer layers with lower cost
- Safety check: max 2-layer jump

### Potential Optimizations

If zigzagging persists after parameter tuning, consider:

1. **Modify DecideLayer thresholds** (requires C++ code changes):
   - Tighten stage 2: 0.2m → 0.15m
   - Tighten stage 3: 0.6m → 0.4m
   - Increase elevation threshold: 0.5 → 0.7

2. **Add hysteresis to layer transitions**:
   - Penalize layer changes in cost function
   - Require minimum distance between transitions

3. **Post-process A* path**:
   - Filter out single-point layer transitions
   - Smooth layer assignments before GPMP

---

## Performance Considerations

**Computation Time vs Quality Tradeoffs:**

| Parameter | ↑ Increase | ↓ Decrease |
|-----------|------------|------------|
| `sample_interval` | Faster, smoother, less accurate | Slower, more accurate, may zigzag |
| `interpolate_num` | Slower, denser trajectory | Faster, sparser trajectory |
| `max_iterations` | Slower, better convergence | Faster, may not converge |
| `qc_position` | Weaker smoothing, faster | Stronger smoothing, slower |

**Typical Timings (example path ~50 waypoints):**
- A* search: 10-100ms
- GPMP optimization: 50-300ms
- Total: 60-400ms

---

## Contact & Support

For issues or questions:
1. Check debug output first
2. Review this guide's troubleshooting section
3. Experiment with recommended parameter ranges
4. Document your findings for future reference

**Good luck optimizing your paths!**
