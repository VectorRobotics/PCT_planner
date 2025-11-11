# Critical Bugfixes Applied

## Issue 1: Optimizer Completing in 0 Iterations with Massive Cost

### Symptom
```
Optimization finished for N = 21 at iteration 0, time elapsed: 5.939000 ms, cost: 3588338347.916672
```

### Root Cause
In `gpmp_optimizer.cc`, the `sigma_initial` and `sigma_goal` variables were declared as `static`, meaning they were initialized only once on the first function call. When the function was called again with different `s_init` or `s_target` values, the sigma matrices still used the old cached values, causing catastrophic optimization failure.

### Fix Applied
**File**: `planner/lib/src/trajectory_optimization/gpmp_optimizer/gpmp_optimizer.cc:44-45`

**Before**:
```cpp
static auto sigma_initial = Diagonal::Sigmas(s_init);
static auto sigma_goal = Diagonal::Sigmas(s_target);
```

**After**:
```cpp
auto sigma_initial = Diagonal::Sigmas(s_init);
auto sigma_goal = Diagonal::Sigmas(s_target);
```

Removed the `static` keyword to ensure sigma matrices are recomputed on each call.

---

## Issue 2: Debug Flags Not Working

### Symptom
Setting `enable_path_debug: true` and `enable_optimizer_debug: true` in the YAML config had no effect. Debug output was not appearing.

### Root Cause
The Python config class (`planner/config/param.py`) didn't include the new debug flag parameters, so `hasattr()` checks in the wrapper were failing and debug flags were never set.

### Fix Applied
**File**: `planner/config/param.py:7-17`

Added the missing parameters to the Python config class:
```python
# Smoothing parameters (GPMP Optimizer)
sample_interval = 10
interpolate_num = 8
max_iterations = 100
lambda_initial = 200.0
qc_position = 0.1
qc_heading = 0.01

# Debug flags
enable_path_debug = True  # Default to True to help with debugging
enable_optimizer_debug = True
```

---

## Issue 3: Debug Output Traceability

### Enhancement Applied
**File**: `planner/scripts/planner_wrapper.py:128-136`

Added diagnostic print statements to confirm debug flags are being loaded:
```python
if hasattr(self.cfg.planner, 'enable_path_debug'):
    enable_path_debug = bool(self.cfg.planner.enable_path_debug)
    print(f"[Config] Setting enable_path_debug: {enable_path_debug}")
    self.planner.set_path_debug(enable_path_debug)
```

This helps diagnose config loading issues.

---

## Testing After Fixes

After applying these fixes and rebuilding, you should see:

### Expected Output (with debug enabled):
```
[Config] Setting enable_path_debug: True
[Config] Setting enable_optimizer_debug: True
[A*] Start searching: start=[...] goal=[...]
[A*] Start height=X.XX, goal height=X.XX
[A*] cost_threshold=XX.X, step_cost_weight=X.XX
[A*] Path found: N waypoints, M iterations, K layer transitions, X.XXms
[A*] Path layer changes: X total, max Y consecutive
[GPMP] === Trajectory Optimization Parameters ===
[GPMP] sample_interval: 10
[GPMP] interpolate_num: 8
...
[GPMP] Optimization finished: N=XX, iterations=XX/100, time=XX.XXms, cost=X.XXXX
```

### Good Convergence Indicators:
- Cost < 50 (ideally < 10)
- Iterations < max_iterations (not hitting the limit)
- No warnings about "Max iterations reached" or "High cost"
- Layer transitions < 20% of total waypoints

---

## Build Commands

```bash
cd /home/alex-lin/autonomy_stack_mecanum_wheel_platform
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select pct_planner
source install/setup.bash
```

---

## Files Modified in This Fix

1. `planner/lib/src/trajectory_optimization/gpmp_optimizer/gpmp_optimizer.cc` - Removed static
2. `planner/config/param.py` - Added debug flags
3. `planner/scripts/planner_wrapper.py` - Added diagnostic prints

---

## Next Steps

1. **Rebuild** the package
2. **Run your planning task** again
3. **Verify** you see the debug output
4. **Check convergence**:
   - Cost should be < 50 (preferably < 10)
   - Should NOT see 0 iterations
   - Should NOT see billion-scale costs

If you still don't see debug output, check that:
- The ROS2 node is loading the YAML config properly
- The config is being passed to the TomogramPlanner constructor
- Python print statements show the config being loaded

---

## Critical Note on Static Variables

This bug is a classic example of why `static` variables in functions can be dangerous:
- They persist across function calls
- They're initialized only once
- Changes to non-static variables don't affect them
- Can cause subtle, hard-to-debug issues

**Lesson**: Avoid `static` for variables that depend on function parameters unless you explicitly want caching behavior with proper invalidation logic.
