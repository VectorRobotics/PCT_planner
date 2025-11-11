#!/usr/bin/env python3

import sys
import os
import pickle
import numpy as np
import open3d as o3d
from typing import Tuple, Optional
from dataclasses import dataclass

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PLANNER_SCRIPTS_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '..', 'planner', 'scripts'))
PLANNER_LIB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '..', 'planner', 'lib'))
TOMOGRAPHY_SCRIPTS_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '..', 'tomography', 'scripts'))

sys.path.insert(0, PLANNER_SCRIPTS_DIR)
sys.path.insert(0, PLANNER_LIB_DIR)
sys.path.insert(0, TOMOGRAPHY_SCRIPTS_DIR)

from planner_wrapper import TomogramPlanner
from utils.vis_ros import traj2ros
from pct_planner.planner.config.param import Config as PlannerConfig
from pct_planner.tomography.scripts.tomogram import Tomogram


@dataclass
class TomogramConfig:
    # Default values match SceneBuilding (scene_building.py)
    resolution: float = 0.10  # SceneMap.resolution
    slice_dh: float = 0.5  # SceneMap.slice_dh
    ground_h: float = 0.0  # SceneMap.ground_h
    kernel_size: int = 7  # SceneTrav.kernel_size
    interval_min: float = 0.50  # SceneTrav.interval_min
    interval_free: float = 0.65  # SceneTrav.interval_free
    slope_max: float = 0.40  # SceneTrav.slope_max (radians, ~22.9 degrees) - SceneBuilding value
    step_max: float = 0.17  # SceneTrav.step_max - SceneBuilding value
    safe_margin: float = 0.4  # SceneTrav.safe_margin
    inflation: float = 0.2  # SceneTrav.inflation
    standable_ratio: float = 0.20  # SceneTrav.standable_ratio
    cost_barrier: float = 50.0  # SceneTrav.cost_barrier


class PCTPlanner:
    def __init__(self, tomogram_dir: Optional[str] = None, tomo_config: Optional[TomogramConfig] = None):
        self.script_dir = SCRIPT_DIR
        self.tomogram_dir = tomogram_dir or os.path.join(
            self.script_dir, '../pct_planner/rsc/tomogram'
        )

        self.planner_cfg = PlannerConfig()
        self.planner_cfg.wrapper.tomo_dir = '/rsc/tomogram/'
        self.planner = TomogramPlanner(self.planner_cfg)

        self.tomo_config = tomo_config if tomo_config is not None else TomogramConfig()
        self.tomogram_processor = None
        self.current_tomogram_file = None
        self.current_metadata = None

    def pointcloud_to_tomogram(
        self,
        points: np.ndarray,
        map_center: Optional[Tuple[float, float]] = None
    ) -> dict:
        if points.shape[0] == 0:
            raise ValueError("Empty point cloud provided")
        if points.shape[1] < 3:
            raise ValueError(f"Points must have at least 3 dimensions, got {points.shape[1]}")

        points = points[:, :3].astype(np.float32)

        # Create Open3D point cloud for cleaning operations
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Statistical outlier removal
        # Removes points that are further away from their neighbors compared to the average
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=10, std_ratio=2.0)

        # Radius outlier removal
        # Removes points that have few neighbors in a given radius
        pcd, _ = pcd.remove_radius_outlier(nb_points=10, radius=0.2)

        # Convert back to numpy array
        points = np.asarray(pcd.points, dtype=np.float32)

        min_xyz = np.min(points, axis=0)
        max_xyz = np.max(points, axis=0)

        if map_center is None:
            map_center = ((min_xyz[:2] + max_xyz[:2]) / 2).astype(np.float32)
        else:
            map_center = np.array(map_center, dtype=np.float32)

        map_dim_x = int(np.ceil((max_xyz[0] - min_xyz[0]) / self.tomo_config.resolution)) + 4
        map_dim_y = int(np.ceil((max_xyz[1] - min_xyz[1]) / self.tomo_config.resolution)) + 4
        n_slice_init = int(np.ceil((max_xyz[2] - min_xyz[2]) / self.tomo_config.slice_dh))
        slice_h0 = min_xyz[2] + self.tomo_config.slice_dh

        class SceneMap:
            pass

        class SceneTrav:
            pass

        class SceneConfig:
            map = SceneMap()
            trav = SceneTrav()

        scene_cfg = SceneConfig()

        # Set map parameters - must match tomography.py exactly
        scene_cfg.map.resolution = self.tomo_config.resolution
        scene_cfg.map.slice_dh = self.tomo_config.slice_dh
        # CRITICAL: ground_h in scene_cfg is NOT used by Tomogram - it's just for reference
        # The actual ground_h was already applied to min_xyz[2] above
        scene_cfg.map.ground_h = self.tomo_config.ground_h

        # Set trav parameters - must match tomography.py exactly
        scene_cfg.trav.kernel_size = self.tomo_config.kernel_size
        scene_cfg.trav.slope_max = self.tomo_config.slope_max
        scene_cfg.trav.step_max = self.tomo_config.step_max
        scene_cfg.trav.interval_min = self.tomo_config.interval_min
        scene_cfg.trav.interval_free = self.tomo_config.interval_free
        scene_cfg.trav.standable_ratio = self.tomo_config.standable_ratio
        scene_cfg.trav.cost_barrier = self.tomo_config.cost_barrier
        scene_cfg.trav.safe_margin = self.tomo_config.safe_margin
        scene_cfg.trav.inflation = self.tomo_config.inflation

        self.tomogram_processor = Tomogram(scene_cfg)
        self.tomogram_processor.initMappingEnv(
            map_center, map_dim_x, map_dim_y, n_slice_init, slice_h0
        )

        layers_t, trav_gx, trav_gy, layers_g, layers_c = \
            self.tomogram_processor.point2map(points)

        tomogram_data = np.stack((layers_t, trav_gx, trav_gy, layers_g, layers_c))

        metadata = {
            'data': tomogram_data,
            'resolution': self.tomo_config.resolution,
            'center': map_center,
            'n_slice': layers_t.shape[0],
            'slice_h0': slice_h0,
            'slice_dh': self.tomo_config.slice_dh,
            'map_dim': [map_dim_x, map_dim_y],
            'bounds': {
                'min': min_xyz,
                'max': max_xyz
            }
        }
        self.current_metadata = metadata
        self.current_tomogram_file = "realtime_tomogram"
        return metadata

    def load_tomogram_direct(self, tomogram_metadata: dict) -> None:
        self.planner.loadTomogramDirect(
            tomogram_metadata['data'],
            tomogram_metadata['resolution'],
            tomogram_metadata['center'],
            tomogram_metadata['slice_h0'],
            tomogram_metadata['slice_dh']
        )
        self.current_metadata = tomogram_metadata

    def load_tomogram(self, tomogram_name: str) -> None:
        self.planner.loadTomogram(tomogram_name)
        self.current_tomogram_file = tomogram_name

        tomogram_path = os.path.join(self.tomogram_dir, f"{tomogram_name}.pickle")
        if os.path.exists(tomogram_path):
            with open(tomogram_path, 'rb') as f:
                self.current_metadata = pickle.load(f)

    def plan_path(
        self,
        start_pose: Tuple[float, float, float],
        goal_pose: Tuple[float, float, float]
    ) -> Optional[np.ndarray]:
        if self.current_tomogram_file is None and self.current_metadata is None:
            raise ValueError("No tomogram loaded")

        start_pos_2d = np.array([start_pose[0], start_pose[1]], dtype=np.float32)
        goal_pos_2d = np.array([goal_pose[0], goal_pose[1]], dtype=np.float32)
        start_z = start_pose[2]
        goal_z = goal_pose[2]

        traj_3d = self.planner.plan(start_pos_2d, goal_pos_2d, start_z, goal_z)
        return traj_3d

    def plan_path_to_ros(
        self,
        start_pose: Tuple[float, float, float],
        goal_pose: Tuple[float, float, float]
    ):
        traj_3d = self.plan_path(start_pose, goal_pose)
        if traj_3d is None:
            return None
        return traj2ros(traj_3d)

    def get_tomogram_info(self) -> Optional[dict]:
        if self.current_metadata is None:
            return None
        return {
            'name': self.current_tomogram_file,
            'resolution': self.current_metadata['resolution'],
            'center': self.current_metadata['center'],
            'dimensions': self.current_metadata['map_dim'],
            'n_slices': self.current_metadata['n_slice'],
            'slice_h0': self.current_metadata['slice_h0'],
            'slice_dh': self.current_metadata['slice_dh'],
            'bounds': self.current_metadata.get('bounds', None)
        }
