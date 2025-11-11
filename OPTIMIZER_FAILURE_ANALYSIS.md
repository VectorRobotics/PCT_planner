# GPMP Optimizer Failure - Deep Investigation

## Problem Summary

The GPMP trajectory optimizer is failing immediately with:
- **Initial error**: 3.25 billion
- **Iterations**: 0 (optimizer gives up immediately)
- **GTSAM error**: "Levenberg-Marquardt giving up because cannot decrease error with maximum lambda"
- **lambda_initial**: 200.0 (user-configured parameter under investigation)

## Root Cause Analysis

### Issue 1: Velocity Initialization Mismatch

**Location**: [gpmp_optimizer.cc:273](src/trajectory_optimization/gpmp_optimizer/gpmp_optimizer.cc#L273)

```cpp
void GPMPOptimizer::PathPointToNode(const PathPoint& path_point, Vector6& x) {
  double v = std::max(path_point.ref_v, 1.0);  // ← FORCES MINIMUM 1.0 m/s!
  x(0, 0) = path_point.x;
  x(1, 0) = std::cos(path_point.heading) * v;
  x(2, 0) = 0.0;
  x(3, 0) = path_point.y;
  x(4, 0) = std::sin(path_point.heading) * v;
  x(5, 0) = 0.0;
}
```

**Problem**:
- A* pathfinder outputs PathPoint with `ref_v = 0.0` (never sets velocity)
- PathPointToNode() forces velocity to minimum 1.0 m/s
- This creates state vectors with large velocities that may not match the actual path geometry

**Evidence**: [a_star_search.cc:419-427](lib/src/a_star/a_star_search.cc#L419-L427)
```cpp
std::vector<PathPoint> Astar::GetPathPoints() const {
  // ...
  for (size_t i = 0; i < size; ++i) {
    path_points[i].layer = search_result_[i]->layer;
    path_points[i].x = search_result_[i]->idx(2);
    path_points[i].y = search_result_[i]->idx(1);
    path_points[i].height = search_result_[i]->height;
    // ... heading is set
    // ref_v is NEVER SET - defaults to 0.0!
  }
}
```

### Issue 2: Extremely Tight Position Priors

**Location**: [gpmp_optimizer.cc:40-45](src/trajectory_optimization/gpmp_optimizer/gpmp_optimizer.cc#L40-L45)

```cpp
Vector6 s_init, s_target;
s_init << 0.001, 0.1, 1, 0.001, 0.1, 1;
s_target << 0.001, 1, 1, 0.001, 1, 1;

auto sigma_initial = Diagonal::Sigmas(s_init);
auto sigma_goal = Diagonal::Sigmas(s_target);
```

**Sigma values breakdown**:
- `[0]` Position x: sigma = **0.001** (EXTREMELY tight)
- `[1]` Velocity x: sigma = 0.1 (start) / 1.0 (goal)
- `[2]` Accel x: sigma = 1.0
- `[3]` Position y: sigma = **0.001** (EXTREMELY tight)
- `[4]` Velocity y: sigma = 0.1 (start) / 1.0 (goal)
- `[5]` Accel y: sigma = 1.0

**Problem**:
- Position constraints have sigma = 0.001, meaning standard deviation of 1 millimeter
- With diagonal noise model, cost = (deviation / sigma)²
- Even tiny position deviations create **massive** penalties
- Example: 1 meter deviation → cost = (1.0 / 0.001)² = 1,000,000

### Issue 3: Conflict Between Constraints

The optimizer faces conflicting requirements:

1. **Position Prior**: Must match x0/xN within 0.001 tolerance
2. **Velocity Prior**: Initial velocity = 0.1, but PathPointToNode forces 1.0
3. **GP Motion Prior**: Expects smooth motion consistent with velocities
4. **Obstacle Factors**: Must avoid collisions
5. **Heading Rate Factors**: Limit angular velocity

With forced velocity of 1.0 m/s and position tolerance of 0.001, the optimizer must:
- Start exactly at x0 (within 1mm)
- Move at 1.0 m/s
- Pass through intermediate waypoints
- End exactly at xN (within 1mm)

This is geometrically impossible if the time intervals (dt) don't perfectly match.

### Issue 4: Lambda Parameter Role

**lambda_initial = 200.0** controls the Levenberg-Marquardt damping:
- Higher lambda → more conservative updates (closer to gradient descent)
- Lower lambda → more aggressive updates (closer to Gauss-Newton)
- When error is huge, even max lambda can't help

**The issue**: Lambda is not the root cause, but it **prevents recovery**:
1. Initial error is ~3 billion (from constraint conflicts above)
2. Optimizer tries to reduce error with lambda = 200.0
3. Error is so large that even tiny steps would increase it
4. Optimizer increases lambda to maximum
5. Still can't decrease error → gives up at iteration 0

## Diagnostic Output Added

Added comprehensive debug output to help diagnose the issue:

```cpp
// Initial/goal state values
[GPMP] === Initial/Goal States ===
[GPMP] x0: [x, vx, ax, y, vy, ay]
[GPMP] xN: [x, vx, ax, y, vy, ay]
[GPMP] path.front(): x=..., y=..., heading=..., ref_v=...
[GPMP] path.back(): x=..., y=..., heading=..., ref_v=...

// Error breakdown
[GPMP] === Initial Error Analysis ===
[GPMP] Total initial error: 3250247408.81
[GPMP] Number of factors: 342
[GPMP] Factor X: error=YYYY (LARGE!)  // Any factor with error > 1000
[GPMP] Approximate breakdown:
[GPMP]   Single-key factors (priors/heading): XXX
[GPMP]   Two-key factors (GP/obstacles): YYY
```

## Proposed Solutions

### Solution 1: Fix Velocity Initialization (RECOMMENDED)

**Option A**: Compute velocity from path geometry
```cpp
void GPMPOptimizer::PathPointToNode(const PathPoint& path_point, Vector6& x,
                                     const PathPoint* next_point, double dt) {
  x(0, 0) = path_point.x;
  x(3, 0) = path_point.y;

  if (next_point != nullptr && dt > 0) {
    // Compute velocity from position difference
    double vx = (next_point->x - path_point.x) / dt;
    double vy = (next_point->y - path_point.y) / dt;
    x(1, 0) = vx;
    x(4, 0) = vy;
  } else {
    // Start/end: use small velocity in heading direction
    double v = 0.1;  // Small velocity instead of 1.0
    x(1, 0) = std::cos(path_point.heading) * v;
    x(4, 0) = std::sin(path_point.heading) * v;
  }
  x(2, 0) = 0.0;
  x(5, 0) = 0.0;
}
```

**Option B**: Use very small velocity
```cpp
double v = 0.1;  // Much smaller than 1.0
```

### Solution 2: Relax Position Priors

**Current**:
```cpp
s_init << 0.001, 0.1, 1, 0.001, 0.1, 1;
s_target << 0.001, 1, 1, 0.001, 1, 1;
```

**Proposed** (relax position constraints by 100x):
```cpp
s_init << 0.1, 0.1, 1, 0.1, 0.1, 1;
s_target << 0.1, 1, 1, 0.1, 1, 1;
```

This allows 10cm deviation instead of 1mm, reducing position penalty by 10,000x.

### Solution 3: Reduce lambda_initial

**Current**: lambda_initial = 200.0

**Proposed**: Try progressively lower values
- lambda = 100.0 (moderate reduction)
- lambda = 50.0 (significant reduction)
- lambda = 10.0 (aggressive, closer to Gauss-Newton)

**Caveat**: This won't fix the root cause but may allow optimizer to make progress.

### Solution 4: Fix A* to Output Velocities

Modify A* to estimate velocity based on path geometry:

```cpp
std::vector<PathPoint> Astar::GetPathPoints() const {
  // ... existing code ...

  // Estimate velocities (assume constant time per step)
  double estimated_v = 0.5;  // m/s, configurable
  for (size_t i = 0; i < size; ++i) {
    if (i > 0 && i < size - 1) {
      double dx = search_result_[i+1]->idx(2) - search_result_[i-1]->idx(2);
      double dy = search_result_[i+1]->idx(1) - search_result_[i-1]->idx(1);
      double dist = std::sqrt(dx*dx + dy*dy) * resolution_;
      path_points[i].ref_v = estimated_v;  // or based on dist
    }
  }
}
```

## Testing Plan

1. **Add diagnostic output**: DONE ✓
2. **Run with current config**: See actual error breakdown
3. **Test Solution 1B**: Change velocity from 1.0 to 0.1
4. **Test Solution 2**: Relax position sigma to 0.1
5. **Test Solution 3**: Reduce lambda_initial to 50.0
6. **Measure results**: Cost, iterations, convergence quality

## Expected Results

If velocity mismatch is the main issue:
- Changing v from 1.0 to 0.1 should reduce initial error by ~100x
- Initial error should drop from 3.25 billion to ~30-300 million

If position priors are the main issue:
- Relaxing sigma from 0.001 to 0.1 reduces penalty by 10,000x
- Initial error should drop from 3.25 billion to ~300,000

Combination of both fixes should get initial error to manageable range (<1000).

## Files Modified

1. [gpmp_optimizer.cc](src/trajectory_optimization/gpmp_optimizer/gpmp_optimizer.cc)
   - Added initial/goal state diagnostics (lines 65-76)
   - Added error breakdown analysis (lines 147-181)

## Next Steps

1. Rebuild and test with diagnostic output
2. Identify which factors contribute most to error
3. Apply fixes in order: velocity → sigma → lambda
4. Validate trajectory quality after fixes
5. Expose sigma values as config parameters
