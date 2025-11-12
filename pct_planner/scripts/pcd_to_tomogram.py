#!/usr/bin/env python3

import os
import sys
import argparse
import pickle
import yaml
import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

# Import PCT planner for tomogram generation
from pct_planner.scripts.pct_planner import PCTPlanner, TomogramConfig


def load_config_from_yaml(yaml_path: str) -> TomogramConfig:
    """Load tomogram configuration from YAML file."""
    with open(yaml_path, 'r') as f:
        params = yaml.safe_load(f)

    # Extract parameters from ROS2 parameter format
    ros_params = params['/**']['ros__parameters']

    return TomogramConfig(
        resolution=ros_params.get('resolution', 0.3),
        slice_dh=ros_params.get('slice_dh', 0.5),
        ground_h=ros_params.get('ground_h', 0.0),
        kernel_size=ros_params.get('kernel_size', 5),
        interval_min=ros_params.get('interval_min', 0.5),
        interval_free=ros_params.get('interval_free', 0.65),
        slope_max=ros_params.get('slope_max', 0.40),
        step_max=ros_params.get('step_max', 0.3),
        safe_margin=ros_params.get('safe_margin', 0.3),
        inflation=ros_params.get('inflation', 0.2),
        standable_ratio=ros_params.get('standable_ratio', 0.5),
        cost_barrier=ros_params.get('cost_barrier', 50.0)
    )


class PointCloudSubscriber(Node):
    """Minimal ROS2 node to receive a single PointCloud2 message."""
    def __init__(self, topic: str):
        super().__init__('pcd_subscriber')
        self.points = None
        self.sub = self.create_subscription(PointCloud2, topic, self.callback, 10)
        self.get_logger().info(f"Waiting for point cloud on {topic}...")

    def callback(self, msg: PointCloud2):
        points_list = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        self.points = np.array(points_list, dtype=np.float32)[:, :3] if isinstance(points_list[0], tuple) else \
                      np.column_stack([np.array(points_list)['x'],
                                      np.array(points_list)['y'],
                                      np.array(points_list)['z']]).astype(np.float32)
        self.get_logger().info(f"Received {len(self.points)} points")
        raise SystemExit  # Exit after receiving one message


def points_to_tomogram(points: np.ndarray, output_path: str, config_file: str = None):
    """Generate tomogram from point cloud array."""
    if len(points) == 0:
        raise ValueError("Point cloud is empty")

    # Load configuration
    if config_file is None:
        config_file = 'src/route_planner/PCT_planner/config/pct_planner_params.yaml'

    print(f"Loading configuration from {config_file}...")
    config = load_config_from_yaml(config_file)
    print(f"Processing {len(points)} points...")

    # Create planner and generate tomogram
    planner = PCTPlanner(tomo_config=config)
    print("Generating tomogram...")
    metadata = planner.pointcloud_to_tomogram(points)

    # Save tomogram
    print(f"Saving tomogram to {output_path}...")
    with open(output_path, 'wb') as f:
        pickle.dump(metadata, f)

    print(f"\nSuccessfully created tomogram: {output_path}")
    print(f"  Resolution: {metadata['resolution']:.3f}m")
    print(f"  Map dimensions: {metadata['map_dim']}")
    print(f"  Number of slices: {metadata['n_slice']}")
    print(f"  Slice height range: {metadata['slice_h0']:.2f}m to {metadata['slice_h0'] + metadata['n_slice'] * metadata['slice_dh']:.2f}m")

    return output_path


def pcd_to_tomogram(pcd_file: str, output_dir: str = None, config_file: str = None):
    """Convert a PCD file to a tomogram pickle file."""
    # Validate input file
    if not os.path.exists(pcd_file):
        raise FileNotFoundError(f"PCD file not found: {pcd_file}")

    if not pcd_file.endswith('.pcd'):
        raise ValueError(f"Input file must be a .pcd file: {pcd_file}")

    # Load point cloud from file
    print(f"Loading point cloud from {pcd_file}...")
    pcd = o3d.io.read_point_cloud(pcd_file)
    points = np.asarray(pcd.points, dtype=np.float32)

    # Prepare output path
    base_name = os.path.splitext(os.path.basename(pcd_file))[0]
    output_name = f"{base_name}_tomogram.pickle"

    if output_dir is None:
        output_dir = os.path.dirname(os.path.abspath(pcd_file))

    output_path = os.path.join(output_dir, output_name)

    return points_to_tomogram(points, output_path, config_file)


def topic_to_tomogram(topic: str, output_path: str, config_file: str = None):
    """Subscribe to ROS2 topic, receive one PointCloud2 message, and convert to tomogram."""
    rclpy.init()
    node = PointCloudSubscriber(topic)

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if node.points is None:
        raise RuntimeError(f"Failed to receive point cloud from {topic}")

    return points_to_tomogram(node.points, output_path, config_file)


def main():
    parser = argparse.ArgumentParser(
        description='Convert PCD file or ROS2 PointCloud2 topic to tomogram pickle file'
    )
    parser.add_argument('input', type=str, nargs='?', default=None,
                       help='Path to input PCD file (omit to use --topic)')
    parser.add_argument('-t', '--topic', type=str, default='/overall_map',
                       help='ROS2 PointCloud2 topic to subscribe to (default: /overall_map)')
    parser.add_argument('-o', '--output', type=str, default=None,
                       help='Output pickle file path (default: input_name_tomogram.pickle or topic_name_tomogram.pickle)')
    parser.add_argument('-c', '--config', type=str, default=None,
                       help='Path to YAML config file (default: src/route_planner/PCT_planner/config/pct_planner_params.yaml)')

    args = parser.parse_args()

    try:
        if args.input:
            # PCD file mode
            output_dir = os.path.dirname(args.output) if args.output else None
            pcd_to_tomogram(args.input, output_dir, args.config)
        else:
            # ROS2 topic mode
            if args.output is None:
                topic_name = args.topic.strip('/').replace('/', '_')
                args.output = f"{topic_name}_tomogram.pickle"
            topic_to_tomogram(args.topic, args.output, args.config)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
