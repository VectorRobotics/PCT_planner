#!/usr/bin/env python3

import sys
import os
import pickle
import numpy as np
from typing import Tuple, Optional
from dataclasses import dataclass

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PLANNER_SCRIPTS_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '..', 'planner', 'scripts'))
PLANNER_LIB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '..', 'planner', 'lib'))
sys.path.insert(0, PLANNER_SCRIPTS_DIR)
sys.path.insert(0, PLANNER_LIB_DIR)

from planner_wrapper import TomogramPlanner
from utils.vis_ros import traj2ros
from pct_planner.planner.config.param import Config as PlannerConfig
from pct_planner.tomography.scripts.tomogram import Tomogram


@dataclass
class TomogramConfig:
    resolution: float = 0.3
    slice_dh: float = 0.5
    ground_h: float = 0.0
    kernel_size: int = 5
    slope_max: float = np.deg2rad(30)
    step_max: float = 0.3
    safe_margin: float = 0.3
    inflation: float = 0.2
    standable_ratio: float = 0.5


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
        min_xyz = np.min(points, axis=0)
        max_xyz = np.max(points, axis=0)

        if map_center is None:
            map_center = ((min_xyz[:2] + max_xyz[:2]) / 2).astype(np.float32)
        else:
            map_center = np.array(map_center, dtype=np.float32)

        map_dim_x = int(np.ceil((max_xyz[0] - min_xyz[0]) / self.tomo_config.resolution)) + 4
        map_dim_y = int(np.ceil((max_xyz[1] - min_xyz[1]) / self.tomo_config.resolution)) + 4
        n_slice_init = int(np.ceil((max_xyz[2] - min_xyz[2]) / self.tomo_config.slice_dh)) + 2
        slice_h0 = self.tomo_config.ground_h if self.tomo_config.ground_h != 0.0 else min_xyz[2]

        class SceneConfig:
            class map:
                resolution = self.tomo_config.resolution
                slice_dh = self.tomo_config.slice_dh
                ground_h = slice_h0

            class trav:
                kernel_size = self.tomo_config.kernel_size
                slope_max = self.tomo_config.slope_max
                step_max = self.tomo_config.step_max
                interval_min = 0.1
                interval_free = 0.2
                standable_ratio = self.tomo_config.standable_ratio
                cost_barrier = 100.0
                safe_margin = self.tomo_config.safe_margin
                inflation = self.tomo_config.inflation

        self.tomogram_processor = Tomogram(SceneConfig())
        self.tomogram_processor.initMappingEnv(
            map_center, map_dim_x, map_dim_y, n_slice_init, slice_h0
        )

        layers_t, trav_gx, trav_gy, layers_g, layers_c, timing = \
            self.tomogram_processor.point2map(points)

        tomogram_data = np.zeros((5, layers_t.shape[0], layers_t.shape[1], layers_t.shape[2]), dtype=np.float32)
        tomogram_data[0] = layers_t
        tomogram_data[1] = trav_gx
        tomogram_data[2] = trav_gy
        tomogram_data[3] = layers_g
        tomogram_data[4] = layers_c

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
        if self.current_tomogram_file is None:
            raise ValueError("No tomogram loaded")

        start_pos_2d = np.array([start_pose[0], start_pose[1]], dtype=np.float32)
        goal_pos_2d = np.array([goal_pose[0], goal_pose[1]], dtype=np.float32)

        traj_3d = self.planner.plan(start_pos_2d, goal_pos_2d)
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
