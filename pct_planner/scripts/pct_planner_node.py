#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
import numpy as np
import threading
from rclpy.wait_for_message import wait_for_message

from std_srvs.srv import Trigger

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

# Import PCT planner and tomogram visualization utilities
from pct_planner.scripts.pct_planner import PCTPlanner, TomogramConfig
from pct_planner.tomography.config.prototype import POINT_FIELDS_XYZI, GRID_POINTS_XYZI
from pct_planner.tomography.scripts.tomogram_viz import generate_tomogram_pointcloud
from pct_planner.utils.path_utils import get_waypoint_from_traj


class PCTPlannerNode(Node):
    def __init__(self):
        super().__init__('pct_planner')

        # Declare localization mode parameter (boolean flag)
        # True = localization mode with pre-loaded tomogram, False = SLAM mode with real-time map building
        self.declare_parameter('local_mode', True)
        self.declare_parameter('tomogram_path', '')  # Required when local_mode=True

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

        # Get parameters
        self.local_mode = self.get_parameter('local_mode').value
        self.tomogram_path = self.get_parameter('tomogram_path').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value

        # Validate parameters
        if self.local_mode and not self.tomogram_path:
            self.get_logger().error("Localization mode (local_mode=True) requires 'tomogram_path' parameter")
            raise ValueError("Missing tomogram_path for localization mode")

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
        self.sub_clicked_point = self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 10)

        # Publishers
        self.pub_path = self.create_publisher(Path, '/global_path', latching_qos)
        self.pub_waypoint = self.create_publisher(PointStamped, '/way_point', 10)
        self.pub_tomogram = self.create_publisher(PointCloud2, '/tomogram', latching_qos)
        self.pub_goal_pose = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pub_goal_marker = self.create_publisher(Marker, '/goal_marker', 10)

        # Mode-specific initialization
        if self.local_mode:
            self._init_relocalization_mode()
        else:
            self._init_slam_mode()

        mode_str = "localization" if self.local_mode else "SLAM"
        self.get_logger().info(f"PCT Planner ready in {mode_str} mode")

    def _init_slam_mode(self):
        """Initialize SLAM mode with service-triggered tomogram building."""
        self.srv_build_tomogram = self.create_service(Trigger, '/build_tomogram', self.build_tomogram_service_callback)
        self.get_logger().info("SLAM mode: Call /build_tomogram (std_srvs/Trigger) to build tomogram from /explored_areas")

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

    def build_tomogram_service_callback(self, request, response):
        """Build a tomogram by getting one message from /explored_areas (SLAM mode only)."""
        del request  # unused
        if self.local_mode:
            response.success = False
            response.message = "Not available in localization mode (local_mode=True)"
            return response

        with self.tomogram_lock:
            if self.tomogram_processing:
                response.success = False
                response.message = "Tomogram build already in progress"
                return response
            self.tomogram_processing = True

        # Start background thread to wait for message and process
        threading.Thread(target=self._build_tomogram_thread, daemon=True).start()

        response.success = True
        response.message = "Tomogram build started, waiting for /explored_areas..."
        return response

    def _build_tomogram_thread(self):
        """Background thread that waits for point cloud and builds tomogram."""
        try:

            self.get_logger().info("Waiting for point cloud on /explored_areas...")
            success, msg = wait_for_message(PointCloud2, self, '/explored_areas', time_to_wait=10.0)

            if not success:
                self.get_logger().error("Timeout waiting for point cloud on /explored_areas")
                self.tomogram_processing = False
                return

            points_list = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            if len(points_list) < 1000:
                self.get_logger().warn(f"Point cloud too small ({len(points_list)} points), skipping")
                self.tomogram_processing = False
                return

            points = np.array(points_list, dtype=np.float32)[:, :3] if isinstance(points_list[0], tuple) else \
                     np.column_stack([np.array(points_list)['x'],
                                     np.array(points_list)['y'],
                                     np.array(points_list)['z']]).astype(np.float32)

            self._process_tomogram_once(points)

        except Exception as e:
            self.get_logger().error(f"Error in tomogram build thread: {e}")
            self.tomogram_processing = False

    def odom_callback(self, msg: Odometry):
        """Odometry callback - updates current pose and publishes waypoints."""
        self.current_odom = msg
        self._publish_waypoint()

    def clicked_point_callback(self, msg: PointStamped):
        """Clicked point callback - converts to goal pose."""
        goal_pose = PoseStamped()
        goal_pose.header = msg.header
        goal_pose.pose.position = msg.point
        goal_pose.pose.orientation.w = 0.0
        self.pub_goal_pose.publish(goal_pose)
        self.get_logger().info(f"Clicked point: ({msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f})")

    def goal_callback(self, msg: PoseStamped):
        """Goal callback - plans path to the received goal pose."""
        goal = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self._plan_path(goal)

    def _process_tomogram_once(self, points_to_process: np.ndarray):
        """Build a single tomogram from the provided point cloud (SLAM mode only)."""
        try:
            self.get_logger().info(f"Building tomogram from {len(points_to_process)} points...")
            metadata = self.planner.pointcloud_to_tomogram(points_to_process)
            self.planner.load_tomogram_direct(metadata)
            map_dim_x, map_dim_y = metadata['map_dim']
            self.VISPROTO_I, self.VISPROTO_P = GRID_POINTS_XYZI(metadata['resolution'], map_dim_x, map_dim_y)
            self.publish_tomogram(metadata)
        except Exception as e:
            self.get_logger().error(f"Failed to build tomogram: {e}")
        finally:
            self.tomogram_processing = False

    def _plan_path(self, goal: tuple):
        """Plan path to goal."""
        if self.current_odom is None:
            self.get_logger().warn("Cannot plan path: missing odometry")
            return
        if self.planner.current_metadata is None:
            self.get_logger().warn("Cannot plan path: no tomogram available")
            return

        try:
            pos = self.current_odom.pose.pose.position
            start_pose = (pos.x, pos.y, pos.z - 0.45)

            path_msg = self.planner.plan_path_to_ros(start_pose, goal)
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

    def _publish_waypoint(self):
        """Extract and publish the next waypoint from the current path."""
        if self.current_path is None or self.current_odom is None:
            return

        try:
            waypoint_msg = get_waypoint_from_traj(
                self.current_path, self.current_odom,
                lookahead_dist=self.lookahead_distance
            )

            if waypoint_msg is not None:
                waypoint_msg.header.stamp = self.get_clock().now().to_msg()
                self.pub_waypoint.publish(waypoint_msg)
        except Exception as e:
            self.get_logger().error(f"Waypoint extraction error: {e}")

    def _publish_goal_marker(self, goal: tuple, frame_id: str = "map"):
        """Publish red sphere marker for goal point."""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = goal[0]
        marker.pose.position.y = goal[1]
        marker.pose.position.z = goal[2]
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.9

        marker.lifetime.sec = 0  # Persistent

        self.pub_goal_marker.publish(marker)

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
