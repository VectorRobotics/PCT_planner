#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
import numpy as np
import threading
import math

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

# Import PCT planner and tomogram visualization utilities
from pct_planner.scripts.pct_planner import PCTPlanner, TomogramConfig
from pct_planner.tomography.config.prototype import POINT_FIELDS_XYZI, GRID_POINTS_XYZI
from pct_planner.tomography.scripts.tomogram_viz import generate_tomogram_pointcloud


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
        self.declare_parameter('slope_max_deg', 30.0)  # degrees
        self.declare_parameter('step_max', 0.3)
        self.declare_parameter('standable_ratio', 0.5)

        # Safety and inflation parameters
        self.declare_parameter('safe_margin', 0.3)
        self.declare_parameter('inflation', 0.2)

        # Get parameters
        resolution = self.get_parameter('resolution').value
        slice_dh = self.get_parameter('slice_dh').value
        ground_h = self.get_parameter('ground_h').value
        kernel_size = self.get_parameter('kernel_size').value
        slope_max_deg = self.get_parameter('slope_max_deg').value
        step_max = self.get_parameter('step_max').value
        standable_ratio = self.get_parameter('standable_ratio').value
        safe_margin = self.get_parameter('safe_margin').value
        inflation = self.get_parameter('inflation').value

        # Create tomogram configuration
        tomo_config = TomogramConfig(
            resolution=resolution,
            slice_dh=slice_dh,
            ground_h=ground_h,
            kernel_size=kernel_size,
            slope_max=np.deg2rad(slope_max_deg),
            step_max=step_max,
            safe_margin=safe_margin,
            inflation=inflation,
            standable_ratio=standable_ratio
        )

        self.current_pose = None
        self.goal_pose = None
        self.point_cloud_map = None
        self.map_updated = False
        self.planning_in_progress = False

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

        # Tomogram publisher with TRANSIENT_LOCAL for visualization
        tomogram_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub_tomogram = self.create_publisher(PointCloud2, '/tomogram', tomogram_qos)

        self.get_logger().info("PCT Planner ready")

    def map_callback(self, msg: PointCloud2):
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

            self.point_cloud_map = points
            self.map_updated = True

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

    def goal_callback(self, msg: PoseStamped):
        self.goal_pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

        if self.point_cloud_map is not None and self.map_updated:
            threading.Thread(target=self._update_tomogram_then_plan, daemon=True).start()
            self.map_updated = False
        else:
            threading.Thread(target=self._plan_path_async, daemon=True).start()

    def _update_tomogram_then_plan(self):
        if self.point_cloud_map is None:
            return

        try:
            tomogram_metadata = self.planner.pointcloud_to_tomogram(
                self.point_cloud_map)
            self.planner.load_tomogram_direct(tomogram_metadata)

            # Store tomogram metadata for visualization
            self.tomogram_center = tomogram_metadata['center']
            self.tomogram_slice_dh = tomogram_metadata['slice_dh']
            map_dim_x, map_dim_y = tomogram_metadata['map_dim']
            resolution = tomogram_metadata['resolution']

            # Generate grid prototypes for visualization
            self.VISPROTO_I, self.VISPROTO_P = GRID_POINTS_XYZI(resolution, map_dim_x, map_dim_y)

            # Publish the tomogram visualization
            self.publish_tomogram(tomogram_metadata)

            self._plan_path_async()

        except Exception as e:
            self.get_logger().error(f"Failed to update tomogram: {e}")

    def _plan_path_async(self):
        if self.planning_in_progress:
            return
        if self.current_pose is None or self.goal_pose is None:
            return
        if self.planner.current_metadata is None:
            return

        self.planning_in_progress = True

        try:
            start_pose = self.current_pose[:3]
            path_msg = self.planner.plan_path_to_ros(start_pose, self.goal_pose)

            if path_msg is not None:
                path_msg.header.frame_id = "map"
                path_msg.header.stamp = self.get_clock().now().to_msg()
                self.pub_path.publish(path_msg)

        except Exception as e:
            self.get_logger().error(f"Path planning error: {e}")

        finally:
            self.planning_in_progress = False

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

            self.get_logger().info(f"Published tomogram with {global_points.shape[0]} points")

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
