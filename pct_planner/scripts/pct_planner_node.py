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

        # Declare parameters for tomogram configuration
        # Map resolution parameters
        self.declare_parameter('resolution', 0.3)
        self.declare_parameter('slice_dh', 0.5)
        self.declare_parameter('ground_h', 0.0)

        # Traversability analysis parameters
        self.declare_parameter('kernel_size', 5)
        self.declare_parameter('interval_min', 0.5)
        self.declare_parameter('interval_free', 0.65)
        self.declare_parameter('slope_max', 0.40)  # radians
        self.declare_parameter('step_max', 0.3)
        self.declare_parameter('standable_ratio', 0.5)
        self.declare_parameter('cost_barrier', 50.0)

        # Safety and inflation parameters
        self.declare_parameter('safe_margin', 0.3)
        self.declare_parameter('inflation', 0.2)

        # Waypoint following parameters
        self.declare_parameter('lookahead_distance', 2.0)
        self.declare_parameter('min_lookahead_distance', 0.5)
        self.declare_parameter('curvature_adaptive', True)
        self.declare_parameter('curvature_scale', 1.0)

        # Get parameters
        resolution = self.get_parameter('resolution').value
        slice_dh = self.get_parameter('slice_dh').value
        ground_h = self.get_parameter('ground_h').value
        kernel_size = self.get_parameter('kernel_size').value
        interval_min = self.get_parameter('interval_min').value
        interval_free = self.get_parameter('interval_free').value
        slope_max = self.get_parameter('slope_max').value
        step_max = self.get_parameter('step_max').value
        standable_ratio = self.get_parameter('standable_ratio').value
        cost_barrier = self.get_parameter('cost_barrier').value
        safe_margin = self.get_parameter('safe_margin').value
        inflation = self.get_parameter('inflation').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.min_lookahead_distance = self.get_parameter('min_lookahead_distance').value
        self.curvature_adaptive = self.get_parameter('curvature_adaptive').value
        self.curvature_scale = self.get_parameter('curvature_scale').value

        # Log the loaded parameters
        self.get_logger().info("PCT Planner Parameters:")
        self.get_logger().info(f"  resolution: {resolution} m")
        self.get_logger().info(f"  slice_dh: {slice_dh} m")
        self.get_logger().info(f"  ground_h: {ground_h} m")
        self.get_logger().info(f"  kernel_size: {kernel_size}")
        self.get_logger().info(f"  interval_min: {interval_min} m")
        self.get_logger().info(f"  interval_free: {interval_free} m")
        self.get_logger().info(f"  slope_max: {slope_max} rad ({np.rad2deg(slope_max):.2f} deg)")
        self.get_logger().info(f"  step_max: {step_max} m")
        self.get_logger().info(f"  standable_ratio: {standable_ratio}")
        self.get_logger().info(f"  cost_barrier: {cost_barrier}")
        self.get_logger().info(f"  safe_margin: {safe_margin} m")
        self.get_logger().info(f"  inflation: {inflation} m")
        self.get_logger().info(f"  lookahead_distance: {self.lookahead_distance} m")
        self.get_logger().info(f"  min_lookahead_distance: {self.min_lookahead_distance} m")
        self.get_logger().info(f"  curvature_adaptive: {self.curvature_adaptive}")
        self.get_logger().info(f"  curvature_scale: {self.curvature_scale}")

        # Create tomogram configuration
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

        self.current_pose = None
        self.current_odom = None
        self.goal_pose = None
        self.current_path = None
        self.point_cloud_map = None
        self.tomogram_processing = False
        self.pending_pointcloud = None
        self.tomogram_lock = threading.Lock()

        # Tomogram visualization variables
        self.VISPROTO_I = None
        self.VISPROTO_P = None
        self.map_frame = "map"
        self.tomogram_center = None
        self.tomogram_slice_dh = None

        self.planner = PCTPlanner(tomo_config=tomo_config)

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        path_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub_map = self.create_subscription(
            PointCloud2, '/explored_areas', self.map_callback, map_qos)
        self.sub_odom = self.create_subscription(
            Odometry, '/state_estimation', self.odom_callback, odom_qos)
        self.sub_goal = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.pub_path = self.create_publisher(Path, '/global_path', path_qos)
        self.pub_waypoint = self.create_publisher(PointStamped, '/way_point', 10)

        # Tomogram publisher with TRANSIENT_LOCAL for visualization
        tomogram_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub_tomogram = self.create_publisher(PointCloud2, '/tomogram', tomogram_qos)

        # Debug occupancy grid publisher
        self.pub_debug_grid = self.create_publisher(OccupancyGrid, '/tomogram_debug_grid', tomogram_qos)

        self.get_logger().info("PCT Planner ready")

    def map_callback(self, msg: PointCloud2):
        """
        Point cloud callback - triggers asynchronous tomogram building.
        Each new point cloud triggers a new tomogram processing task.
        """
        try:
            points_list = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

            if len(points_list) < 1000:
                return

            if len(points_list) > 0 and isinstance(points_list[0], tuple):
                points = np.array(points_list, dtype=np.float32)
            else:
                points_array = np.array(points_list)
                points = np.zeros((len(points_array), 3), dtype=np.float32)
                points[:, 0] = points_array['x']
                points[:, 1] = points_array['y']
                points[:, 2] = points_array['z']

            # Store the new point cloud
            with self.tomogram_lock:
                self.pending_pointcloud = points

            # Start processing if not already processing
            if not self.tomogram_processing:
                threading.Thread(target=self._process_tomogram, daemon=True).start()

        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {e}")

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        quat = msg.pose.pose.orientation
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.current_pose = (x, y, z, yaw)
        self.current_odom = msg

        # Publish waypoint if we have a path
        self._publish_waypoint()

    def goal_callback(self, msg: PoseStamped):
        """Goal callback - triggers synchronous path planning."""
        self.goal_pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self._plan_path()

    def _process_tomogram(self):
        """
        Continuously process tomograms using the latest point cloud.
        """
        self.tomogram_processing = True

        while True:
            # Get the latest pending point cloud
            with self.tomogram_lock:
                points_to_process = self.pending_pointcloud
                self.pending_pointcloud = None

            # If no pending point cloud, exit the loop
            if points_to_process is None:
                break

            try:
                self.get_logger().info(f"Building tomogram from {len(points_to_process)} points...")

                # Build the tomogram (this is the slow part)
                tomogram_metadata = self.planner.pointcloud_to_tomogram(points_to_process)
                self.planner.load_tomogram_direct(tomogram_metadata)

                # Store the processed point cloud map
                self.point_cloud_map = points_to_process

                # Store tomogram metadata for visualization
                self.tomogram_center = tomogram_metadata['center']
                self.tomogram_slice_dh = tomogram_metadata['slice_dh']
                map_dim_x, map_dim_y = tomogram_metadata['map_dim']
                resolution = tomogram_metadata['resolution']

                # Generate grid prototypes for visualization
                self.VISPROTO_I, self.VISPROTO_P = GRID_POINTS_XYZI(resolution, map_dim_x, map_dim_y)

                # Publish the tomogram visualization
                self.publish_tomogram(tomogram_metadata)

            except Exception as e:
                self.get_logger().error(f"Failed to build tomogram: {e}")

            # After finishing, check if there's a new point cloud to process
            # (the while loop will continue if pending_pointcloud was updated during processing)

        self.tomogram_processing = False

    def _plan_path(self):
        """Plan path synchronously (fast operation)."""
        if self.current_pose is None or self.goal_pose is None:
            self.get_logger().warn("Cannot plan path: missing pose or goal")
            return
        if self.planner.current_metadata is None:
            self.get_logger().warn("Cannot plan path: no tomogram available")
            return

        try:
            start_pose = self.current_pose[:3]
            adjusted_goal = self.goal_pose

            # Validate and adjust goal to safe location
            metadata = self.planner.current_metadata
            layers_t = metadata['data'][0]  # traversability cost (inflated cost)
            n_slices, dim_x, dim_y = layers_t.shape

            # Select layer for goal height
            layer_idx = select_layer_for_height(
                self.goal_pose[2],
                metadata['slice_h0'],
                metadata['slice_dh'],
                n_slices
            )

            if layer_idx is not None:
                # Convert tomogram layer to occupancy grid
                # layers_t[layer_idx] shape is (dim_x, dim_y)
                layer_cost = layers_t[layer_idx]
                occupancy_grid = tomogram_to_occupancy_grid(
                    layer_cost,
                    metadata['resolution'],
                    metadata['center']
                )

                # Publish debug grid
                occupancy_grid.header.frame_id = "map"
                occupancy_grid.header.stamp = self.get_clock().now().to_msg()
                self.pub_debug_grid.publish(occupancy_grid)

                # Find safe goal using BFS
                safe_goal_2d = find_safe_goal_bfs(
                    costmap=occupancy_grid,
                    goal=(self.goal_pose[0], self.goal_pose[1]),
                    cost_threshold=50,  # Cost threshold (0-100)
                    min_clearance=0.5,  # Minimum clearance in meters
                    max_search_distance=5.0,  # Max search distance in meters
                    connectivity_check_radius=3  # Connectivity check radius in cells
                )

                if safe_goal_2d is not None:
                    # Use the adjusted 2D position with the layer height
                    layer_height = metadata['slice_h0'] + layer_idx * metadata['slice_dh']
                    adjusted_goal = (safe_goal_2d[0], safe_goal_2d[1], layer_height)

                    # Log adjustment
                    dist = np.linalg.norm(np.array(self.goal_pose[:2]) - np.array(safe_goal_2d))
                    if dist > 0.01:
                        self.get_logger().info(f"Goal adjusted by {dist:.2f}m to safe location")
                else:
                    self.get_logger().warn("Could not find safe goal, using original")
            else:
                self.get_logger().warn(f"Goal height {self.goal_pose[2]:.2f}m out of tomogram range")

            # Plan path with adjusted goal
            path_msg = self.planner.plan_path_to_ros(start_pose, adjusted_goal)

            if path_msg is not None:
                path_msg.header.frame_id = "map"
                path_msg.header.stamp = self.get_clock().now().to_msg()
                self.pub_path.publish(path_msg)
                self.current_path = path_msg
                self.get_logger().info(f"Published path with {len(path_msg.poses)} poses")

                # Immediately publish first waypoint
                self._publish_waypoint()
            else:
                self.get_logger().warn("Path planning failed")
                self.current_path = None

        except Exception as e:
            self.get_logger().error(f"Path planning error: {e}")

    def _publish_waypoint(self):
        """Extract and publish the next waypoint from the current path."""
        if self.current_path is None or self.current_odom is None:
            return

        try:
            waypoint_msg = get_waypoint_from_traj(
                self.current_path,
                self.current_odom,
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

    def publish_tomogram(self, tomogram_metadata: dict):
        """
        Publish the tomogram as a PointCloud2 message for visualization.
        Uses the shared visualization utility from tomogram_viz module.
        """
        if self.VISPROTO_I is None or self.VISPROTO_P is None:
            return

        try:
            # Extract tomogram layers from metadata
            tomogram_data = tomogram_metadata['data']
            layers_t = tomogram_data[0]  # Traversability layer
            layers_g = tomogram_data[3]  # Geometry layer

            # Use the shared visualization utility function
            global_points = generate_tomogram_pointcloud(
                layers_g, layers_t,
                self.VISPROTO_I, self.VISPROTO_P,
                self.tomogram_center, self.tomogram_slice_dh
            )

            # Create and publish the PointCloud2 message
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.map_frame

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
