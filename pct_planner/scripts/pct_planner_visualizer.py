#!/usr/bin/env python3
"""
PCT Planner Visualizer Node

This node loads a PCD file, processes it through the PCT planner tomogram generation,
and publishes the resulting tomogram visualization for viewing in RViz.

Similar to tomography.py but uses the PCTPlanner class for consistency.
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
import numpy as np

try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
except ImportError:
    OPEN3D_AVAILABLE = False

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

from pct_planner.scripts.pct_planner import PCTPlanner, TomogramConfig
from pct_planner.tomography.config.prototype import POINT_FIELDS_XYZI, GRID_POINTS_XYZI
from pct_planner.tomography.scripts.tomogram_viz import generate_tomogram_pointcloud


class PCTPlannerVisualizer(Node):
    def __init__(self):
        super().__init__('pct_planner_visualizer')

        if not OPEN3D_AVAILABLE:
            self.get_logger().error("Open3D is not installed. Please install it with: pip install open3d")
            raise ImportError("Open3D is required for PCD file loading")

        # Declare parameter for PCD file path
        self.declare_parameter('pcd_file', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('publish_rate', 1.0)  # Hz
        self.declare_parameter('publish_layers', False)

        # Declare tomogram configuration parameters (same as pct_planner_node)
        self.declare_parameter('resolution', 0.3)
        self.declare_parameter('slice_dh', 0.5)
        self.declare_parameter('ground_h', 0.0)
        self.declare_parameter('kernel_size', 5)
        self.declare_parameter('interval_min', 0.5)
        self.declare_parameter('interval_free', 0.65)
        self.declare_parameter('slope_max', 0.40)  # radians
        self.declare_parameter('step_max', 0.3)
        self.declare_parameter('standable_ratio', 0.5)
        self.declare_parameter('cost_barrier', 50.0)
        self.declare_parameter('safe_margin', 0.3)
        self.declare_parameter('inflation', 0.2)

        # Get parameters
        pcd_file = self.get_parameter('pcd_file').value
        self.map_frame = self.get_parameter('map_frame').value
        publish_rate = self.get_parameter('publish_rate').value
        self.publish_layers = self.get_parameter('publish_layers').value

        if not pcd_file:
            self.get_logger().error("No PCD file specified! Use parameter 'pcd_file'")
            raise ValueError("pcd_file parameter is required")

        # Get tomogram config parameters
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

        # Log parameters
        self.get_logger().info("PCT Planner Visualizer Parameters:")
        self.get_logger().info(f"  PCD file: {pcd_file}")
        self.get_logger().info(f"  resolution: {resolution} m")
        self.get_logger().info(f"  slice_dh: {slice_dh} m")
        self.get_logger().info(f"  kernel_size: {kernel_size}")
        self.get_logger().info(f"  slope_max: {slope_max} rad ({np.rad2deg(slope_max):.2f} deg)")
        self.get_logger().info(f"  safe_margin: {safe_margin} m")
        self.get_logger().info(f"  inflation: {inflation} m")

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

        # Create planner
        self.planner = PCTPlanner(tomo_config=tomo_config)

        # Set up publishers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pub_tomogram = self.create_publisher(PointCloud2, '/tomogram', qos_profile)
        self.pub_raw_cloud = self.create_publisher(PointCloud2, '/raw_pointcloud', qos_profile)

        if self.publish_layers:
            self.layer_g_pubs = []
            self.layer_c_pubs = []

        # Load and process PCD file
        self.get_logger().info(f"Loading PCD file: {pcd_file}")
        points = self.load_pcd(pcd_file)

        if points is None or points.shape[0] == 0:
            self.get_logger().error("Failed to load PCD file or file is empty")
            raise ValueError("Failed to load PCD file")

        self.get_logger().info(f"Loaded {points.shape[0]} points")

        # Process through tomogram
        self.get_logger().info("Processing tomogram...")
        try:
            tomogram_metadata = self.planner.pointcloud_to_tomogram(points)
            self.tomogram_metadata = tomogram_metadata
            self.raw_points = points

            # Debug: Check tomogram costs
            tomogram_data = tomogram_metadata['data']
            layers_t = tomogram_data[0]
            self.get_logger().info(f"Tomogram cost analysis:")
            self.get_logger().info(f"  layers_t shape: {layers_t.shape}")
            self.get_logger().info(f"  layers_t min/max: {np.nanmin(layers_t):.2f} / {np.nanmax(layers_t):.2f}")
            self.get_logger().info(f"  Non-zero cost cells: {np.sum(layers_t > 0.1)} / {layers_t.size}")
            self.get_logger().info(f"  High cost cells (>10): {np.sum(layers_t > 10)}")
            self.get_logger().info(f"  Very high cost cells (>40): {np.sum(layers_t > 40)}")

            # Generate grid prototypes for visualization
            map_dim_x, map_dim_y = tomogram_metadata['map_dim']
            self.VISPROTO_I, self.VISPROTO_P = GRID_POINTS_XYZI(
                tomogram_metadata['resolution'], map_dim_x, map_dim_y
            )

            n_slices = tomogram_metadata['n_slice']
            self.get_logger().info(f"Tomogram generated with {n_slices} slices")

            # Create layer publishers if requested
            if self.publish_layers:
                for i in range(n_slices):
                    layer_g_pub = self.create_publisher(
                        PointCloud2, f'/tomogram_layer_geometry_{i}', qos_profile
                    )
                    self.layer_g_pubs.append(layer_g_pub)
                    layer_c_pub = self.create_publisher(
                        PointCloud2, f'/tomogram_layer_cost_{i}', qos_profile
                    )
                    self.layer_c_pubs.append(layer_c_pub)

        except Exception as e:
            self.get_logger().error(f"Failed to process tomogram: {e}")
            raise

        # Create timer for periodic publishing
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_callback)

        self.get_logger().info("PCT Planner Visualizer ready - publishing tomogram")

    def load_pcd(self, pcd_file):
        """Load a PCD file using Open3D."""
        if not os.path.exists(pcd_file):
            self.get_logger().error(f"PCD file not found: {pcd_file}")
            return None

        try:
            pcd = o3d.io.read_point_cloud(pcd_file)
            points = np.asarray(pcd.points).astype(np.float32)

            if points.shape[1] > 3:
                points = points[:, :3]

            return points

        except Exception as e:
            self.get_logger().error(f"Error loading PCD file: {e}")
            return None

    def publish_callback(self):
        """Publish the tomogram and raw point cloud."""
        timestamp = self.get_clock().now().to_msg()
        header = Header()
        header.stamp = timestamp
        header.frame_id = self.map_frame

        # Publish raw point cloud
        try:
            raw_msg = pc2.create_cloud_xyz32(header, self.raw_points)
            self.pub_raw_cloud.publish(raw_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing raw cloud: {e}")

        # Publish tomogram
        try:
            tomogram_data = self.tomogram_metadata['data']
            layers_t = tomogram_data[0]  # Traversability
            layers_g = tomogram_data[3]  # Geometry

            # Generate visualization point cloud
            global_points = generate_tomogram_pointcloud(
                layers_g, layers_t,
                self.VISPROTO_I, self.VISPROTO_P,
                self.tomogram_metadata['center'],
                self.tomogram_metadata['slice_dh']
            )

            tomogram_msg = pc2.create_cloud(header, POINT_FIELDS_XYZI, global_points)
            self.pub_tomogram.publish(tomogram_msg)

        except Exception as e:
            self.get_logger().error(f"Error publishing tomogram: {e}")

        # Publish individual layers if enabled
        if self.publish_layers:
            try:
                self.publish_individual_layers(header)
            except Exception as e:
                self.get_logger().error(f"Error publishing layers: {e}")

    def publish_individual_layers(self, header):
        """Publish individual tomogram layers for detailed visualization."""
        tomogram_data = self.tomogram_metadata['data']
        layers_t = tomogram_data[0]
        layers_g = tomogram_data[3]
        layers_c = tomogram_data[4]

        layer_points = self.VISPROTO_P.copy()
        layer_points[:, :2] += self.tomogram_metadata['center']

        for i in range(self.tomogram_metadata['n_slice']):
            # Geometry layer
            layer_points[:, 2] = layers_g[i, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
            layer_points[:, 3] = layers_t[i, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
            valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]

            if valid_points.shape[0] > 0:
                msg = pc2.create_cloud(header, POINT_FIELDS_XYZI, valid_points)
                self.layer_g_pubs[i].publish(msg)

            # Cost layer
            layer_points[:, 2] = layers_c[i, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
            layer_points[:, 3] = 1.0
            valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]

            if valid_points.shape[0] > 0:
                msg = pc2.create_cloud(header, POINT_FIELDS_XYZI, valid_points)
                self.layer_c_pubs[i].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PCTPlannerVisualizer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ValueError, ImportError) as e:
        if isinstance(e, (ValueError, ImportError)):
            print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
