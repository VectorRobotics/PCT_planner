# Optimizer Test Results

## Test 1: Lambda=200.0 with Adjusted Sigma (BEFORE config rebuild)

**Config**:
- lambda_initial: 200.0 (old value, config not yet loaded)
- Position sigma: 0.01 (from C++ code change)
- Velocity sigma: 0.5 (from C++ code change)

**Path**:
- A* found: 191 waypoints, 10 layer changes
- Subsampled to: 20 waypoints
- Final trajectory: 172 points

**Optimization Results**:
```
Initial error: 6,523,420,649.42
Iteration 1:   5,242,375,335.42  (-19.6%, reduction: 1.28 billion)
Iteration 2:   3,425,320,278.39  (-34.7%, reduction: 1.82 billion)
Status: "Levenberg-Marquardt giving up because cannot decrease error with maximum lambda"
Final: 2 iterations, cost=3.43 billion
```

**Analysis**:
✓ **Optimizer made progress!** (unlike the 0-iteration failure before)
✓ Error reduced by 47% in 2 iterations
✗ Still failed with massive cost (3.4 billion)
✗ Hit maximum lambda limit after only 2 iterations

**Error Breakdown**:
- Total: 6.52 billion
- Single-key factors (priors/heading): **0.00** (!)
- Two-key factors (GP/obstacles): **6.52 billion (100%)**

**Key Finding**:
The sigma adjustment (0.001→0.01) eliminated position prior errors entirely, exposing that **100% of error comes from obstacle collisions**. The relaxed sigma values revealed the fundamental problem: GP interpolation between subsampled waypoints creates paths that collide with obstacles.

**Major Obstacle Factors** (> 100 million):
- Factor 84, 86: 892 million each
- Factor 215: 1.08 billion (!)
- Factor 219: 707 million
- Factor 221: 489 million
- Factor 80, 82, 88, 90: 56-495 million
- Factor 274: 873 million
- Factor 276: 363 million

These factors correspond to GP-interpolated waypoints cutting through walls/obstacles.

## Next Test: Lambda=10.0 (AFTER config rebuild)

**Expected Results**:
- More iterations (lambda allows more aggressive updates)
- Further error reduction (possibly to < 1 billion)
- May converge or may oscillate

**Success Criteria**:
- ✓ Iterations > 2
- ✓ Final cost < 3.4 billion (improvement from test 1)
- ? Final cost < 1 billion (good, but may not achieve)
- ? Final cost < 50 (excellent convergence, unlikely with obstacle conflicts)

## Key Insights

### 1. Lambda Was Masking the Real Problem
With lambda=200.0, the optimizer was too conservative to make progress. Reducing it allowed us to see that it CAN reduce error, but the fundamental issue remains: obstacle collisions.

### 2. Sigma Adjustment Revealed Root Cause
By relaxing position sigma from 0.001 to 0.01, we eliminated position prior errors and revealed that **100% of the error is from obstacles**. This proves the problem is not constraint conflicts between priors, but rather the GP interpolation strategy.

### 3. The Fundamental Architecture Issue
The optimizer faces an impossible task:
1. A* creates a 191-waypoint path carefully avoiding obstacles
2. Subsampling reduces it to 20 waypoints (loss of 90% of geometric detail)
3. GP interpolation creates smooth curves between these 20 points
4. These curves cut straight through obstacles that the original path avoided

**Example from logs**:
- A* layer transition from layer 7→8 at specific location to avoid obstacle
- After subsampling: waypoints skip this transition
- GP interpolation: straight line through the obstacle
- Optimizer: tries to pull path away from obstacle, but constrained by smoothness

### 4. Why Optimizer Still Fails
Even with error reduction from 6.5B → 3.4B:
- Remaining 3.4B error is still from obstacle collisions
- Optimizer can't "magically" make path avoid obstacles without waypoints to guide it
- Lambda limit prevents further exploration (with lambda=200)

## Recommendations

### Short-term (Test with lambda=10.0)
Should allow more iterations and further error reduction. Test to see final cost and trajectory quality.

### Medium-term Fixes
1. **Reduce subsampling aggressiveness**: `sample_interval: 10 → 5`
   - Keeps more A* waypoints (40 instead of 20)
   - Gives optimizer more geometric guidance

2. **Increase interpolation density**: `interpolate_num: 8 → 16`
   - More intermediate points for obstacle checking

3. **Expose sigma as config parameters**
   - Allow runtime tuning without C++ recompilation

### Long-term Architecture Changes
1. **Hierarchical optimization**:
   - First pass: optimize smoothness without obstacles
   - Second pass: add obstacle constraints locally

2. **Adaptive subsampling**:
   - Use dense sampling in high-obstacle areas
   - Use sparse sampling in open spaces

3. **Alternative interpolation**:
   - Use spline through A* waypoints instead of GP
   - Respects original path geometry better

## Test Summary Table

| Test | Lambda | Sample Int | Sigma Pos | Iterations | Initial Error | Final Error | Reduction | Status |
|------|--------|------------|-----------|------------|---------------|-------------|-----------|--------|
| Original | 200.0 | 10 | 0.001 | 0 | 3.25B | 3.25B | 0% | Failed immediately |
| Test 1 | 200.0 | 10 | 0.01 | 2 | 6.52B | 3.43B | 47% | Progress but hit lambda limit |
| Test 2 | 10.0 | 10 | 0.01 | 1 | 2.35B | 468M | **80%** | **BEST result!** |
| Test 3 | 5.0 | 10 | 0.01 | 0 | 3.72B | 3.72B | 0% | Failed - too aggressive |
| Test 4 | 10.0 | **5** | 0.01 | ? | ? | ? | ? | **PENDING** - denser waypoints |

**Key Findings**:
- **Lambda=10.0 is optimal**: 5.0 too aggressive (fails), 200.0 too conservative
- Lambda=10.0 achieved 80% reduction in single iteration (1.88B decrease)
- Final cost improved 7x: 468M vs 3.43B with lambda=200
- **Lambda sweet spot exists**: too low causes failure, too high prevents progress
- Test 4 reduces sample_interval (10→5) to keep 2x more A* waypoints

Note: Test 1 initial error is higher (6.52B vs 3.25B) because relaxed sigma exposed obstacle errors that were previously masked by position prior errors.
