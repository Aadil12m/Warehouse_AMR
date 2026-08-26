#!/usr/bin/env python3
"""Computer-vision rack detection and waypoint publishing.

Reads the SLAM-generated map (.yaml + .pgm), detects storage racks with OpenCV,
converts pixel centroids to map-frame coordinates, applies a stand-off offset,
and publishes them once as a PoseArray on /rack_poses.

Real-robot notes:
  - All magic numbers from the Gazebo warehouse (rack grid extents, contour
    area window, stand-off distance, facing yaw) are now ROS parameters so the
    same node works on hardware without code edits.
  - Publisher uses TRANSIENT_LOCAL durability: a late-starting Behavior Tree
    subscriber still receives the poses. Previously the single latched publish
    could be missed entirely if the BT wasn't up yet -> mission hung forever.
"""
import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseArray, Pose
import yaml


class map_operation(Node):
    def __init__(self):
        super().__init__('map_operation_node')

        # ---- Parameters (defaults = Gazebo warehouse values) -----------------
        self.declare_parameter('map_path',
            '/home/aadil/AMR_ws/src/Warehouse_AMR/robot_gazebo/maps/my_new_map')
        self.declare_parameter('min_contour_area', 20.0)
        self.declare_parameter('max_contour_area', 500.0)
        self.declare_parameter('grid_min_x', -1.3)
        self.declare_parameter('grid_max_x', 1.3)
        self.declare_parameter('grid_min_y', -1.3)
        self.declare_parameter('grid_max_y', 1.3)
        self.declare_parameter('standoff_distance', 0.55)   # m south of rack centre
        self.declare_parameter('approach_yaw_deg', 90.0)    # face +Y toward rack

        # ---- Publisher: reliable + transient_local (latched) -----------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub_ = self.create_publisher(PoseArray, 'rack_poses', qos)
        self.has_published = False
        self.timer_ = self.create_timer(1.0, self.pub_callback)

        self.get_logger().info(
            f"Waiting for map at {self.get_parameter('map_path').value} ...")

    def pub_callback(self):
        map_path = self.get_parameter('map_path').value

        # Load the YAML file to get dynamic map metadata
        try:
            with open(f'{map_path}.yaml', 'r') as file:
                map_data = yaml.safe_load(file)
                resolution = map_data['resolution']
                origin_x = map_data['origin'][0]
                origin_y = map_data['origin'][1]
        except Exception:
            # We don't error out because the map might not be saved yet!
            self.get_logger().info("Waiting for Explore node to save the map...", throttle_duration_sec=5.0)
            return

        map_img = cv2.imread(f'{map_path}.pgm', cv2.IMREAD_GRAYSCALE)
        if map_img is None:
            self.get_logger().error("YAML found but could not load .pgm image!")
            return

        min_area = self.get_parameter('min_contour_area').value
        max_area = self.get_parameter('max_contour_area').value
        grid_min_x = self.get_parameter('grid_min_x').value
        grid_max_x = self.get_parameter('grid_max_x').value
        grid_min_y = self.get_parameter('grid_min_y').value
        grid_max_y = self.get_parameter('grid_max_y').value
        standoff = self.get_parameter('standoff_distance').value
        approach_yaw = np.deg2rad(self.get_parameter('approach_yaw_deg').value)

        coords = []
        _, thresh = cv2.threshold(map_img, 240, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)

            if area < min_area or area > max_area:
                continue

            M = cv2.moments(c)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # Dynamically calculate the real-world coordinates!
                map_height = map_img.shape[0]
                world_x = (cX * resolution) + origin_x
                world_y = ((map_height - cY) * resolution) + origin_y

                # Filter: only accept detections inside the configured rack area.
                # Tune per environment via parameters (Gazebo default: +/-1.3 m).
                if not (grid_min_x <= world_x <= grid_max_x and
                        grid_min_y <= world_y <= grid_max_y):
                    self.get_logger().info(
                        f"REJECTED: area={area:.1f} world=({world_x:.2f}, {world_y:.2f}) - outside rack grid")
                    continue

                waypoint_x = world_x
                waypoint_y = world_y - standoff

                coords.append([waypoint_x, waypoint_y])

                self.get_logger().info(
                    f"Rack Pixel ({cX},{cY}) -> World ({world_x:.2f}, {world_y:.2f}). "
                    f"Waypoint: ({waypoint_x:.2f}, {waypoint_y:.2f})")

        Poses = PoseArray()
        Poses.header.frame_id = "map"
        Poses.header.stamp = self.get_clock().now().to_msg()

        yaw_z = float(np.sin(approach_yaw / 2.0))
        yaw_w = float(np.cos(approach_yaw / 2.0))
        for coord in coords:
            rack_waypoint = Pose()
            rack_waypoint.position.x = float(coord[0])
            rack_waypoint.position.y = float(coord[1])
            # Orientation: approach_yaw (default 90 deg = face +Y, toward the rack)
            rack_waypoint.orientation.z = yaw_z
            rack_waypoint.orientation.w = yaw_w

            Poses.poses.append(rack_waypoint)

        if not self.has_published:
            self.pub_.publish(Poses)
            self.get_logger().info(f"Published {len(coords)} rack poses (transient_local).")
            self.has_published = True

            # Optional: destroy the timer so we don't keep running OpenCV
            self.timer_.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = map_operation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
