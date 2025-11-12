#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
import numpy as np
import threading
import math

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Header

# Import PCT planner and tomogram visualization utilities
from pct_planner.scripts.pct_planner import PCTPlanner, TomogramConfig
from pct_planner.tomography.config.prototype import POINT_FIELDS_XYZI, GRID_POINTS_XYZI
from pct_planner.tomography.scripts.tomogram_viz import generate_tomogram_pointcloud
from pct_planner.utils.goal_validator import tomogram_to_occupancy_grid, select_layer_for_height, find_safe_goal_bfs
from pct_planner.utils.path_utils import get_waypoint_from_traj


class PCTPlannerNode(Node):
    def __init__(self):
        super().__init__('pct_planner')

        # Declare mode parameter (slam or relocalization)
        self.declare_parameter('mode', 'slam')
        self.declare_parameter('tomogram_path', '')  # Required for relocalization mode

        # Declare tomogram configuration parameters
        self.declare_parameter('resolution', 0.3)
        self.declare_parameter('slice_dh', 0.5)
        self.declare_parameter('ground_h', 0.0)
        self.declare_parameter('kernel_size', 5)
        self.declare_parameter('interval_min', 0.5)
        self.declare_parameter('interval_free', 0.65)
        self.declare_parameter('slope_max', 0.40)
        self.declare_parameter('step_max', 0.3)
        self.declare_parameter('standable_ratio', 0.5)
        self.declare_parameter('cost_barrier', 50.0)
        self.declare_parameter('safe_margin', 0.3)
        self.declare_parameter('inflation', 0.2)

        # Waypoint following parameters
        self.declare_parameter('lookahead_distance', 2.0)
        self.declare_parameter('min_lookahead_distance', 0.5)
        self.declare_parameter('curvature_adaptive', True)
        self.declare_parameter('curvature_scale', 1.0)

        # Get mode and parameters
        self.mode = self.get_parameter('mode').value
        self.tomogram_path = self.get_parameter('tomogram_path').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.min_lookahead_distance = self.get_parameter('min_lookahead_distance').value
        self.curvature_adaptive = self.get_parameter('curvature_adaptive').value
        self.curvature_scale = self.get_parameter('curvature_scale').value

        # Validate mode
        if self.mode not in ['slam', 'relocalization']:
            self.get_logger().error(f"Invalid mode '{self.mode}'. Must be 'slam' or 'relocalization'")
            raise ValueError(f"Invalid mode: {self.mode}")

        if self.mode == 'relocalization' and not self.tomogram_path:
            self.get_logger().error("Relocalization mode requires 'tomogram_path' parameter")
            raise ValueError("Missing tomogram_path for relocalization mode")

        # Create tomogram configuration
        tomo_config = TomogramConfig(
            resolution=self.get_parameter('resolution').value,
            slice_dh=self.get_parameter('slice_dh').value,
            ground_h=self.get_parameter('ground_h').value,
            kernel_size=self.get_parameter('kernel_size').value,
            interval_min=self.get_parameter('interval_min').value,
            interval_free=self.get_parameter('interval_free').value,
            slope_max=self.get_parameter('slope_max').value,
            step_max=self.get_parameter('step_max').value,
            safe_margin=self.get_parameter('safe_margin').value,
            inflation=self.get_parameter('inflation').value,
            standable_ratio=self.get_parameter('standable_ratio').value,
            cost_barrier=self.get_parameter('cost_barrier').value
        )

        # State variables
        self.current_odom = None
        self.current_path = None
        self.tomogram_processing = False
        self.pending_pointcloud = None
        self.tomogram_lock = threading.Lock()
        self.VISPROTO_I = None
        self.VISPROTO_P = None

        # Initialize planner
        self.planner = PCTPlanner(tomo_config=tomo_config)

        # QoS profiles
        latching_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers (common to both modes)
        self.sub_odom = self.create_subscription(Odometry, '/state_estimation', self.odom_callback, odom_qos)
        self.sub_goal = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Publishers
        self.pub_path = self.create_publisher(Path, '/global_path', latching_qos)
        self.pub_waypoint = self.create_publisher(PointStamped, '/way_point', 10)
        self.pub_tomogram = self.create_publisher(PointCloud2, '/tomogram', latching_qos)
        self.pub_debug_grid = self.create_publisher(OccupancyGrid, '/tomogram_debug_grid', latching_qos)

        # Mode-specific initialization
        if self.mode == 'slam':
            self._init_slam_mode()
        else:
            self._init_relocalization_mode()

        self.get_logger().info(f"PCT Planner ready in {self.mode} mode")

    def _init_slam_mode(self):
        """Initialize SLAM mode with map subscriber."""
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.sub_map = self.create_subscription(PointCloud2, '/explored_areas', self.map_callback, map_qos)
        self.get_logger().info("SLAM mode: Subscribed to /explored_areas for real-time tomogram building")

    def _init_relocalization_mode(self):
        """Initialize relocalization mode with pre-loaded tomogram."""
        try:
            self.get_logger().info(f"Loading tomogram from: {self.tomogram_path}")
            self.planner.load_tomogram(self.tomogram_path)

            if self.planner.current_metadata is not None:
                metadata = self.planner.current_metadata
                map_dim_x, map_dim_y = metadata['map_dim']
                self.VISPROTO_I, self.VISPROTO_P = GRID_POINTS_XYZI(metadata['resolution'], map_dim_x, map_dim_y)
                self.publish_tomogram(metadata)
                self.tomogram_timer = self.create_timer(0.5, lambda: self.publish_tomogram(metadata))
            else:
                self.get_logger().error("Failed to load tomogram metadata")
        except Exception as e:
            self.get_logger().error(f"Failed to load tomogram: {e}")
            raise

    def map_callback(self, msg: PointCloud2):
        """Point cloud callback - triggers asynchronous tomogram building (SLAM mode only)."""
        try:
            points_list = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            if len(points_list) < 1000:
                return

            points = np.array(points_list, dtype=np.float32)[:, :3] if isinstance(points_list[0], tuple) else \
                     np.column_stack([np.array(points_list)['x'],
                                     np.array(points_list)['y'],
                                     np.array(points_list)['z']]).astype(np.float32)

            with self.tomogram_lock:
                self.pending_pointcloud = points

            if not self.tomogram_processing:
                threading.Thread(target=self._process_tomogram, daemon=True).start()

        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {e}")

    def odom_callback(self, msg: Odometry):
        """Odometry callback - updates current pose and publishes waypoints."""
        self.current_odom = msg
        self._publish_waypoint()

    def goal_callback(self, msg: PoseStamped):
        """Goal callback - triggers path planning."""
        goal = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self._plan_path(goal)

    def _process_tomogram(self):
        """Continuously process tomograms using the latest point cloud (SLAM mode only)."""
        self.tomogram_processing = True

        while True:
            with self.tomogram_lock:
                points_to_process = self.pending_pointcloud
                self.pending_pointcloud = None

            if points_to_process is None:
                break

            try:
                self.get_logger().info(f"Building tomogram from {len(points_to_process)} points...")
                metadata = self.planner.pointcloud_to_tomogram(points_to_process)
                self.planner.load_tomogram_direct(metadata)
                map_dim_x, map_dim_y = metadata['map_dim']
                self.VISPROTO_I, self.VISPROTO_P = GRID_POINTS_XYZI(metadata['resolution'], map_dim_x, map_dim_y)
                self.publish_tomogram(metadata)
            except Exception as e:
                self.get_logger().error(f"Failed to build tomogram: {e}")

        self.tomogram_processing = False

    def _plan_path(self, goal: tuple):
        """Plan path to goal with automatic goal adjustment to safe location."""
        if self.current_odom is None:
            self.get_logger().warn("Cannot plan path: missing odometry")
            return
        if self.planner.current_metadata is None:
            self.get_logger().warn("Cannot plan path: no tomogram available")
            return

        try:
            # Extract start pose
            pos = self.current_odom.pose.pose.position
            start_pose = (pos.x, pos.y, pos.z)

            # Adjust goal to safe location
            adjusted_goal = self._find_safe_goal(goal)

            # Plan path
            path_msg = self.planner.plan_path_to_ros(start_pose, adjusted_goal)
            if path_msg is not None:
                path_msg.header.frame_id = "map"
                path_msg.header.stamp = self.get_clock().now().to_msg()
                self.pub_path.publish(path_msg)
                self.current_path = path_msg
                self.get_logger().info(f"Published path with {len(path_msg.poses)} poses")
                self._publish_waypoint()
            else:
                self.get_logger().warn("Path planning failed")
                self.current_path = None

        except Exception as e:
            self.get_logger().error(f"Path planning error: {e}")

    def _find_safe_goal(self, goal: tuple) -> tuple:
        """Find a safe goal location near the requested goal."""
        metadata = self.planner.current_metadata
        layers_t = metadata['data'][0]
        n_slices = layers_t.shape[0]

        layer_idx = select_layer_for_height(goal[2], metadata['slice_h0'], metadata['slice_dh'], n_slices)
        if layer_idx is None:
            self.get_logger().warn(f"Goal height {goal[2]:.2f}m out of range, using original")
            return goal

        # Convert layer to occupancy grid
        occupancy_grid = tomogram_to_occupancy_grid(layers_t[layer_idx], metadata['resolution'], metadata['center'])
        occupancy_grid.header.frame_id = "map"
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        self.pub_debug_grid.publish(occupancy_grid)

        # Find safe goal
        safe_goal_2d = find_safe_goal_bfs(
            costmap=occupancy_grid,
            goal=(goal[0], goal[1]),
            cost_threshold=50,
            min_clearance=0.5,
            max_search_distance=5.0,
            connectivity_check_radius=3
        )

        if safe_goal_2d is not None:
            layer_height = metadata['slice_h0'] + layer_idx * metadata['slice_dh']
            adjusted = (safe_goal_2d[0], safe_goal_2d[1], layer_height)
            dist = np.linalg.norm(np.array(goal[:2]) - np.array(safe_goal_2d))
            if dist > 0.01:
                self.get_logger().info(f"Goal adjusted by {dist:.2f}m to safe location")
            return adjusted
        else:
            self.get_logger().warn("Could not find safe goal, using original")
            return goal

    def _publish_waypoint(self):
        """Extract and publish the next waypoint from the current path."""
        if self.current_path is None or self.current_odom is None:
            return

        try:
            waypoint_msg = get_waypoint_from_traj(
                self.current_path, self.current_odom,
                lookahead_dist=self.lookahead_distance,
                min_lookahead_dist=self.min_lookahead_distance,
                curvature_adaptive=self.curvature_adaptive,
                curvature_scale=self.curvature_scale
            )

            if waypoint_msg is not None:
                waypoint_msg.header.stamp = self.get_clock().now().to_msg()
                self.pub_waypoint.publish(waypoint_msg)
        except Exception as e:
            self.get_logger().error(f"Waypoint extraction error: {e}")

    def publish_tomogram(self, metadata: dict):
        """Publish tomogram as PointCloud2 for visualization."""
        if self.VISPROTO_I is None or self.VISPROTO_P is None:
            return

        try:
            layers_t, layers_g = metadata['data'][0], metadata['data'][3]
            global_points = generate_tomogram_pointcloud(
                layers_g, layers_t, self.VISPROTO_I, self.VISPROTO_P,
                metadata['center'], metadata['slice_dh']
            )

            header = Header(stamp=self.get_clock().now().to_msg(), frame_id="map")
            points_msg = pc2.create_cloud(header, POINT_FIELDS_XYZI, global_points)
            self.pub_tomogram.publish(points_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish tomogram: {e}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PCTPlannerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
