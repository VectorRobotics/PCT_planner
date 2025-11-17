#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
import numpy as np
import threading

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Header
from std_srvs.srv import Trigger

# Import PCT planner and tomogram visualization utilities
from pct_planner.scripts.pct_planner import PCTPlanner, TomogramConfig
from pct_planner.tomography.config.prototype import POINT_FIELDS_XYZI, GRID_POINTS_XYZI
from pct_planner.tomography.scripts.tomogram_viz import generate_tomogram_pointcloud
from pct_planner.utils.goal_validator import tomogram_to_occupancy_grid, select_layer_for_height, find_safe_goal_bfs
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
        self.pub_debug_grid = self.create_publisher(OccupancyGrid, '/tomogram_debug_grid', latching_qos)
        self.pub_goal_pose = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Mode-specific initialization
        if self.local_mode:
            self._init_relocalization_mode()
        else:
            self._init_slam_mode()

        mode_str = "localization" if self.local_mode else "SLAM"
        self.get_logger().info(f"PCT Planner ready in {mode_str} mode")

    def _init_slam_mode(self):
        """Initialize SLAM mode with service for manual tomogram building."""
        # Subscribe to point cloud topic and cache latest message
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.declare_parameter('map_topic', '/explored_areas')
        self.map_topic = self.get_parameter('map_topic').value

        self.latest_pointcloud = None
        self.pointcloud_lock = threading.Lock()
        self.sub_map = self.create_subscription(
            PointCloud2,
            self.map_topic,
            self._cache_pointcloud_callback,
            map_qos
        )

        self.srv_build_tomogram = self.create_service(
            Trigger,
            '~/build_tomogram',
            self.build_tomogram_callback
        )
        self.get_logger().info(f"SLAM mode: Service ~/build_tomogram ready (using topic: {self.map_topic})")

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

    def _cache_pointcloud_callback(self, msg: PointCloud2):
        """Cache the latest point cloud message."""
        with self.pointcloud_lock:
            self.latest_pointcloud = msg

    def build_tomogram_callback(self, _request, response):
        """Service callback to manually build tomogram from cached point cloud."""
        try:
            self.get_logger().info(f"Building tomogram from cached point cloud")

            # Get cached point cloud
            with self.pointcloud_lock:
                pointcloud_msg = self.latest_pointcloud

            if pointcloud_msg is None:
                response.success = False
                response.message = f"No point cloud received yet on {self.map_topic}"
                self.get_logger().error(response.message)
                return response

            # Process point cloud
            points_list = list(pc2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True))
            if len(points_list) < 1000:
                response.success = False
                response.message = f"Insufficient points: {len(points_list)} < 1000"
                self.get_logger().warn(response.message)
                return response

            points = np.array(points_list, dtype=np.float32)[:, :3] if isinstance(points_list[0], tuple) else \
                     np.column_stack([np.array(points_list)['x'],
                                     np.array(points_list)['y'],
                                     np.array(points_list)['z']]).astype(np.float32)

            # Build tomogram
            self.get_logger().info(f"Building tomogram from {len(points)} points...")
            metadata = self.planner.pointcloud_to_tomogram(points)
            self.planner.load_tomogram_direct(metadata)
            map_dim_x, map_dim_y = metadata['map_dim']
            self.VISPROTO_I, self.VISPROTO_P = GRID_POINTS_XYZI(metadata['resolution'], map_dim_x, map_dim_y)
            self.publish_tomogram(metadata)

            response.success = True
            response.message = f"Successfully built tomogram from {len(points)} points"
            self.get_logger().info(response.message)
            return response

        except Exception as e:
            response.success = False
            response.message = f"Failed to build tomogram: {str(e)}"
            self.get_logger().error(response.message)
            return response

    def odom_callback(self, msg: Odometry):
        """Odometry callback - updates current pose and publishes waypoints."""
        self.current_odom = msg
        self._publish_waypoint()

    def clicked_point_callback(self, msg: PointStamped):
        """Clicked point callback - converts to goal pose with no orientation."""
        goal_pose = PoseStamped()
        goal_pose.header = msg.header
        goal_pose.pose.position = msg.point
        # Set orientation to all zeros to indicate no goal yaw
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 0.0
        self.pub_goal_pose.publish(goal_pose)
        self.get_logger().info(f"Clicked point converted to goal: ({msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f})")

    def goal_callback(self, msg: PoseStamped):
        """Goal callback - triggers path planning."""
        goal = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self._plan_path(goal)

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
                lookahead_dist=self.lookahead_distance
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
