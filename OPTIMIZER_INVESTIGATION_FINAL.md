# GPMP Optimizer Failure - Final Investigation Summary

## Executive Summary

The optimizer was failing with **9.67 billion initial error** and 0 iterations. After deep investigation, I found the root cause is **NOT lambda_initial: 200.0**, but rather:

1. **Obstacle collisions in GP interpolation** (9.3B out of 9.67B error)
2. **Overly tight position constraints** masking the real problem
3. **Lambda too high to allow recovery** from the massive error

## Investigation Timeline

### Initial Symptoms
```
Initial error: 3.25 billion (original test)
Initial error: 9.67 billion (after first fix attempt)
Iterations: 0
GTSAM: "Levenberg-Marquardt giving up because cannot decrease error with maximum lambda"
```

### Root Cause Discovery

#### Attempt 1: Relax Position Sigma (0.001 → 0.1)
**Result**: Made problem WORSE (error increased 3.25B → 9.67B)

**Why it failed**:
- Original sigma=0.001 created position penalties so large they dominated everything
- This **masked** the real problem: obstacle collisions
- When relaxed to sigma=0.1, position errors dropped 10,000x
- This **exposed** the massive obstacle collision errors

#### Error Breakdown (from debug output)
```
Total error: 9,672,620,443.80
├─ Single-key factors (priors/heading): 28,424,505.54 (0.3%)
└─ Two-key factors (GP/obstacles):   9,338,636,531.40 (96.5%)

Massive obstacle factor errors:
- Factor 103, 105, 291: 892 million each
- Factor 236: 1.4 billion
- Factor 238: 1.3 billion
- Factor 289: 1.2 billion
- Factor 107, 219, 234, 280, 287: 60-670 million each
```

**Conclusion**: 96.5% of error comes from obstacle collisions!

### The Real Problem

The A* pathfinder creates a path with 193 waypoints and 11 layer changes. The optimizer then:

1. **Subsamples** to 20 waypoints (sample_interval=10)
2. **GP-interpolates** smooth curves between waypoints
3. These smooth curves **cut through obstacles** that the zigzag A* path avoided

Example from log:
```
A*: 193 waypoints with careful layer transitions
Subsampled: 20 waypoints
GP interpolated: 172 points total
Result: Smooth curves pass through walls/obstacles
```

The GP interpolation creates geometrically smooth trajectories that violate obstacle constraints because it doesn't respect the discrete nature of the multi-layer environment.

## Applied Fixes

### Fix 1: Moderate Position Sigma
```cpp
// Before: 0.001 (too tight) or 0.1 (too loose)
// After: 0.01 (1cm tolerance - balanced)
s_init << 0.01, 0.5, 1, 0.01, 0.5, 1;
s_target << 0.01, 0.5, 1, 0.01, 0.5, 1;
```

Changes:
- Position sigma: 0.001 → 0.01 (10x more flexible, still tight)
- Velocity sigma: 0.1/1.0 → 0.5 (uniform, more flexible)

### Fix 2: Reduce Lambda Initial
```yaml
# config/pct_planner_params.yaml
lambda_initial: 10.0  # Was 200.0
```

**Why this helps**:
- Lambda=200 makes optimizer very conservative
- With 9.67B initial error, even tiny steps can't reduce error
- Lambda=10 allows more aggressive Gauss-Newton-like updates
- Optimizer can potentially "push through" obstacles to find better path

### Fix 3: Velocity Handling
```cpp
// Use smaller default velocity when A* doesn't provide it
double v = (path_point.ref_v > 0.01) ? path_point.ref_v : 0.1;
```

### Fix 4: Comprehensive Diagnostics
Added detailed error breakdown to identify which factors contribute most to total error.

## Testing Results Expected

With lambda=10.0 instead of 200.0, we expect:
- **Optimizer will take iterations** (not give up at 0)
- **Error may not fully converge** due to obstacle conflicts
- **Path quality**: Uncertain - may still have issues if optimizer can't resolve obstacle collisions

### Success Criteria
✓ Iterations > 0 (optimizer makes progress)
✓ Final cost decreases from initial (shows improvement)
? Final cost < 50 (good convergence) - may not achieve due to obstacle conflicts

### Failure Modes
⚠️ If still fails at 0 iterations: Lambda still too high, try lambda=5.0 or 1.0
⚠️ If converges but cost >1000: Obstacle collisions remain, need better initial guess
⚠️ If oscillates without converging: Lambda too low, increase slightly

## Fundamental Issue Remains

**The core problem is architectural**:

A* plans in discrete 3D grid with layer transitions. GPMP optimizes continuous smooth curves. When you subsample a discrete path and interpolate smoothly, you lose the geometric constraints that made the original path feasible.

###Long-term Solutions (Not Implemented)

1. **Reduce subsampling aggressiveness**
   - Use sample_interval=3-5 instead of 10
   - Keeps more A* waypoints to guide optimizer

2. **Add more obstacle factors**
   - Check obstacles along interpolated segments
   - Penalize any collision points

3. **Multi-stage optimization**
   - First optimize without obstacle factors (get smooth path)
   - Then add obstacles and refine locally

4. **Better initial guess**
   - Use spline through A* waypoints instead of GP interpolation
   - Respects original path geometry better

5. **Hybrid approach**
   - Keep A* path in high-obstacle areas
   - Only smooth in open spaces

## Files Modified

1. **[gpmp_optimizer.cc](lib/src/trajectory_optimization/gpmp_optimizer/gpmp_optimizer.cc)**
   - Lines 41-45: Adjusted sigma values (0.001→0.01 position, 0.5 velocity)
   - Lines 65-78: Added initial/goal state diagnostics
   - Lines 128-162: Added error breakdown analysis
   - Lines 205-216: Fixed velocity initialization (1.0→0.1 default)

2. **[pct_planner_params.yaml](config/pct_planner_params.yaml)**
   - Line 120: Reduced lambda_initial from 200.0 to 10.0

3. **[OPTIMIZER_FAILURE_ANALYSIS.md](OPTIMIZER_FAILURE_ANALYSIS.md)** (previous doc)
   - Initial investigation findings

## Next Steps

1. **Test with lambda=10.0** and observe:
   - Does optimizer make iterations?
   - Does error decrease?
   - What's the final trajectory quality?

2. **If still fails**, progressively reduce lambda:
   - Try 5.0, then 1.0

3. **If converges with high cost**, try:
   - Reduce sample_interval (10→5) for denser waypoints
   - Increase safe_cost_margin to penalize near-obstacles more

4. **Consider** exposing sigma values as config parameters for runtime tuning

## Key Insight

**Lambda was a symptom, not the cause.** The real problem is that GP interpolation between subsampled A* waypoints creates paths that violate obstacle constraints. Lambda=200 just prevented the optimizer from even trying to fix it.

Reducing lambda to 10.0 gives the optimizer a chance to work around obstacles, but fundamental geometric conflicts may remain depending on the environment complexity.
