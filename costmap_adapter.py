#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import Costmap, CostmapMetaData
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


class OccupancyToCostmap(Node):
    """Convert the simulator's raw occupancy map into a planner costmap."""

    def __init__(self):
        super().__init__("sim_costmap_adapter")

        # unimportant for now
        self.declare_parameter("robot_radius_m", 0.15)
        self.declare_parameter("unknown_is_obstacle", False)
        self.declare_parameter("rolling_window", False)
        self.declare_parameter("window_width_m", 30.0)
        self.declare_parameter("window_height_m", 30.0)
        self.declare_parameter("update_rate_hz", 10.0)
        self.robot_radius_m = float(self.get_parameter("robot_radius_m").value)
        self.unknown_is_obstacle = bool(self.get_parameter("unknown_is_obstacle").value)
        self.rolling_window = bool(self.get_parameter("rolling_window").value)
        self.window_width_m = float(self.get_parameter("window_width_m").value)
        self.window_height_m = float(self.get_parameter("window_height_m").value)
        update_rate_hz = float(self.get_parameter("update_rate_hz").value)
        self.latest_map = None
        self.latest_pose = (0.0, 0.0)
        self.last_report_ns = 0

        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.publisher = self.create_publisher(Costmap, "/costmap_raw", map_qos)
        self.subscription = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, map_qos
        )
        self.pose_subscription = self.create_subscription(
            PoseStamped, "/sim_pose", self.pose_callback, 10
        )
        self.timer = self.create_timer(1.0 / update_rate_hz, self.publish_window)
        self.get_logger().info(
            f"Publishing /costmap_raw with {self.robot_radius_m:.2f} m inflation, "
            f"rolling_window={self.rolling_window}"
        )

    def map_callback(self, occupancy: OccupancyGrid):
        self.latest_map = occupancy
        self.get_logger().info(
            f"Received map: {occupancy.info.width}x{occupancy.info.height} "
            f"at {occupancy.info.resolution:.3f} m/cell"
        )

    def pose_callback(self, pose: PoseStamped):
        self.latest_pose = (pose.pose.position.x, pose.pose.position.y)

    def publish_window(self):
        occupancy = self.latest_map
        if occupancy is None:
            return
        width = occupancy.info.width
        height = occupancy.info.height
        resolution = occupancy.info.resolution

        if width == 0 or height == 0 or resolution <= 0.0:
            self.get_logger().warning("Ignoring empty or invalid occupancy grid")
            return
        if len(occupancy.data) != width * height:
            self.get_logger().warning("Ignoring occupancy grid with invalid data length")
            return

        source_origin_x = occupancy.info.origin.position.x
        source_origin_y = occupancy.info.origin.position.y
        vehicle_x, vehicle_y = self.latest_pose
        if self.rolling_window:
            window_width = max(1, int(round(self.window_width_m / resolution)))
            window_height = max(1, int(round(self.window_height_m / resolution)))
            window_origin_x = vehicle_x - (window_width * resolution) / 2.0
            window_origin_y = vehicle_y - (window_height * resolution) / 2.0
        else:
            window_width = width
            window_height = height
            window_origin_x = source_origin_x
            window_origin_y = source_origin_y

        raw_window = []
        for row in range(window_height):
            for column in range(window_width):
                world_x = window_origin_x + (column + 0.5) * resolution
                world_y = window_origin_y + (row + 0.5) * resolution
                source_column = int(math.floor((world_x - source_origin_x) / resolution))
                source_row = int(math.floor((world_y - source_origin_y) / resolution))
                if 0 <= source_column < width and 0 <= source_row < height:
                    raw_window.append(occupancy.data[source_row * width + source_column])
                else:
                    raw_window.append(-1)

        inflation_cells = int(math.ceil(self.robot_radius_m / resolution))
        occupied = [
            value >= 100 or (value < 0 and self.unknown_is_obstacle)
            for value in raw_window
        ]
        cost_data = [255 if value < 0 and self.unknown_is_obstacle else 0 for value in raw_window]

        for row in range(window_height):
            for column in range(window_width):
                index = row * window_width + column
                if not occupied[index]:
                    continue

                row_start = max(0, row - inflation_cells)
                row_end = min(window_height, row + inflation_cells + 1)
                column_start = max(0, column - inflation_cells)
                column_end = min(window_width, column + inflation_cells + 1)

                for inflated_row in range(row_start, row_end):
                    for inflated_column in range(column_start, column_end):
                        distance = math.hypot(
                            inflated_row - row, inflated_column - column
                        )
                        if distance <= inflation_cells:
                            inflated_index = inflated_row * window_width + inflated_column
                            cost_data[inflated_index] = 254

        costmap = Costmap()
        costmap.header = occupancy.header
        metadata = CostmapMetaData()
        metadata.map_load_time = occupancy.info.map_load_time
        metadata.update_time = occupancy.header.stamp
        metadata.size_x = window_width
        metadata.size_y = window_height
        metadata.resolution = resolution
        metadata.origin = occupancy.info.origin
        metadata.origin.position.x = window_origin_x
        metadata.origin.position.y = window_origin_y
        costmap.metadata = metadata
        costmap.data = cost_data
        if rclpy.ok():
            self.publisher.publish(costmap)

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_report_ns >= 1_000_000_000:
            raw_obstacle_cells = sum(value >= 100 for value in raw_window)
            lethal_cells = sum(value >= 253 for value in cost_data)
            self.get_logger().info(
                f"Costmap update: center=({vehicle_x:.2f}, {vehicle_y:.2f}) "
                f"origin=({window_origin_x:.2f}, {window_origin_y:.2f}) "
                f"inflation={self.robot_radius_m:.2f} m "
                f"raw_obstacle_cells={raw_obstacle_cells} lethal_cells={lethal_cells}"
            )
            self.last_report_ns = now_ns


def main():
    rclpy.init()
    node = OccupancyToCostmap()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()