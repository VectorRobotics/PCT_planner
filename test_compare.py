#!/usr/bin/env python3
"""
Direct comparison test between tomography.py and pct_planner.py
Run this to see exact differences in output
"""

import sys
import os
import numpy as np
import open3d as o3d

# Add paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
TOMOGRAPHY_SCRIPTS_DIR = os.path.join(SCRIPT_DIR, 'pct_planner/tomography/scripts')
PLANNER_SCRIPTS_DIR = os.path.join(SCRIPT_DIR, 'pct_planner/planner/scripts')
PLANNER_LIB_DIR = os.path.join(SCRIPT_DIR, 'pct_planner/planner/lib')
PCT_SCRIPTS_DIR = os.path.join(SCRIPT_DIR, 'pct_planner/scripts')

sys.path.insert(0, TOMOGRAPHY_SCRIPTS_DIR)
sys.path.insert(0, PLANNER_SCRIPTS_DIR)
sys.path.insert(0, PLANNER_LIB_DIR)
sys.path.insert(0, PCT_SCRIPTS_DIR)
sys.path.insert(0, SCRIPT_DIR)  # For pct_planner package imports

from tomogram import Tomogram

# Import PCTPlanner components manually to avoid package import issues
import importlib.util
spec = importlib.util.spec_from_file_location("pct_planner_module",
    os.path.join(PCT_SCRIPTS_DIR, "pct_planner.py"))
pct_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pct_module)
PCTPlanner = pct_module.PCTPlanner
TomogramConfig = pct_module.TomogramConfig

# Load test PCD - MUST match SceneBuilding
pcd_file = "pct_planner/rsc/pcd/building2_9.pcd"  # SceneBuilding uses this file
pcd_path = os.path.join(SCRIPT_DIR, pcd_file)

if not os.path.exists(pcd_path):
    print(f"ERROR: PCD file not found: {pcd_path}")
    print("Please specify a valid PCD file path")
    sys.exit(1)

pcd = o3d.io.read_point_cloud(pcd_path)
points = np.asarray(pcd.points).astype(np.float32)

print(f"=== Loaded {points.shape[0]} points from {pcd_file} ===\n")

# Common configuration matching SceneBuilding (NOT default scene.py!)
resolution = 0.10
slice_dh = 0.5
ground_h = 0.0
kernel_size = 7
interval_min = 0.50
interval_free = 0.65
slope_max = 0.40  # SceneBuilding uses 0.40, not 0.36!
step_max = 0.17   # SceneBuilding uses 0.17, not 0.20!
standable_ratio = 0.20
cost_barrier = 50.0
safe_margin = 0.4
inflation = 0.2

print("=== Configuration ===")
print(f"resolution: {resolution}")
print(f"slice_dh: {slice_dh}")
print(f"ground_h: {ground_h}")
print(f"kernel_size: {kernel_size}")
print(f"interval_min: {interval_min}")
print(f"interval_free: {interval_free}")
print(f"slope_max: {slope_max}")
print(f"step_max: {step_max}")
print(f"standable_ratio: {standable_ratio}")
print(f"cost_barrier: {cost_barrier}")
print(f"safe_margin: {safe_margin}")
print(f"inflation: {inflation}")
print()

# Test 1: Direct Tomogram usage (like tomography.py)
print("=== Test 1: Direct Tomogram (like tomography.py) ===")

class SceneMap:
    pass

class SceneTrav:
    pass

class SceneConfig:
    map = SceneMap()
    trav = SceneTrav()

scene_cfg = SceneConfig()
scene_cfg.map.resolution = resolution
scene_cfg.map.slice_dh = slice_dh
scene_cfg.map.ground_h = ground_h
scene_cfg.trav.kernel_size = kernel_size
scene_cfg.trav.slope_max = slope_max
scene_cfg.trav.step_max = step_max
scene_cfg.trav.interval_min = interval_min
scene_cfg.trav.interval_free = interval_free
scene_cfg.trav.standable_ratio = standable_ratio
scene_cfg.trav.cost_barrier = cost_barrier
scene_cfg.trav.safe_margin = safe_margin
scene_cfg.trav.inflation = inflation

tomogram1 = Tomogram(scene_cfg)

min_xyz = np.min(points, axis=0)
max_xyz = np.max(points, axis=0)
min_xyz[2] = ground_h

map_center = ((min_xyz[:2] + max_xyz[:2]) / 2).astype(np.float32)
map_dim_x = int(np.ceil((max_xyz[0] - min_xyz[0]) / resolution)) + 4
map_dim_y = int(np.ceil((max_xyz[1] - min_xyz[1]) / resolution)) + 4
n_slice_init = int(np.ceil((max_xyz[2] - min_xyz[2]) / slice_dh))
slice_h0 = min_xyz[2] + slice_dh

tomogram1.initMappingEnv(map_center, map_dim_x, map_dim_y, n_slice_init, slice_h0)
layers_t1, trav_gx1, trav_gy1, layers_g1, layers_c1, t1 = tomogram1.point2map(points)

print(f"Output shapes: {layers_t1.shape}, {layers_g1.shape}")
print(f"layers_t1 min/max/mean: {np.nanmin(layers_t1):.3f} / {np.nanmax(layers_t1):.3f} / {np.nanmean(layers_t1):.3f}")
print(f"Non-zero costs: {np.sum(layers_t1 > 0)}")
print(f"Barrier costs: {np.sum(layers_t1 >= cost_barrier)}")
print()

# Test 2: Through PCT Planner
print("=== Test 2: Through PCTPlanner ===")

tomo_config = TomogramConfig(
    resolution=resolution,
    slice_dh=slice_dh,
    ground_h=ground_h,
    kernel_size=kernel_size,
    interval_min=interval_min,
    interval_free=interval_free,
    slope_max=slope_max,
    step_max=step_max,
    safe_margin=safe_margin,
    inflation=inflation,
    standable_ratio=standable_ratio,
    cost_barrier=cost_barrier
)

planner = PCTPlanner(tomo_config=tomo_config)
metadata = planner.pointcloud_to_tomogram(points, map_center=map_center)

layers_t2 = metadata['data'][0]
layers_g2 = metadata['data'][3]

print(f"Output shapes: {layers_t2.shape}, {layers_g2.shape}")
print(f"layers_t2 min/max/mean: {np.nanmin(layers_t2):.3f} / {np.nanmax(layers_t2):.3f} / {np.nanmean(layers_t2):.3f}")
print(f"Non-zero costs: {np.sum(layers_t2 > 0)}")
print(f"Barrier costs: {np.sum(layers_t2 >= cost_barrier)}")
print()

# Compare
print("=== Comparison ===")
print(f"Shape match: {layers_t1.shape == layers_t2.shape}")
print(f"Arrays equal: {np.allclose(layers_t1, layers_t2, equal_nan=True)}")
if not np.allclose(layers_t1, layers_t2, equal_nan=True):
    diff = np.abs(layers_t1 - layers_t2)
    diff[np.isnan(diff)] = 0
    print(f"Max difference: {np.max(diff):.6f}")
    print(f"Mean difference: {np.mean(diff):.6f}")
    print(f"Num different cells: {np.sum(diff > 0.001)}")
