#!/usr/bin/env python3

import math

import rclpy
from cev_msgs.msg import Trajectory, Waypoint
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException


class AStarPathVisualizer(Node):
    """Adapt the planner's standard Path into the simulator's drawable trajectory."""

    def __init__(self):
        super().__init__("astar_path_visualizer")
        self.publisher = self.create_publisher(Trajectory, "/trajectory", 10)
        self.subscription = self.create_subscription(
            Path, "/astar_path", self.path_callback, 10
        )
        self.costmap = None
        self.costmap_subscription = self.create_subscription(
            Costmap, "/costmap_raw", self.costmap_callback, 10
        )
        self.get_logger().info("Visualizing /astar_path on the simulator as /trajectory")

    def costmap_callback(self, costmap: Costmap):
        self.costmap = costmap

    def blocked_waypoints(self, path: Path):
        if self.costmap is None:
            return None
        metadata = self.costmap.metadata
        blocked = 0
        for pose in path.poses:
            column = int(math.floor((pose.pose.position.x - metadata.origin.position.x) / metadata.resolution))
            row = int(math.floor((pose.pose.position.y - metadata.origin.position.y) / metadata.resolution))
            if (
                row < 0
                or column < 0
                or row >= metadata.size_y
                or column >= metadata.size_x
                or self.costmap.data[row * metadata.size_x + column] >= 253
            ):
                blocked += 1
        return blocked

    def path_callback(self, path: Path):
        trajectory = Trajectory()
        for pose in path.poses:
            waypoint = Waypoint()
            waypoint.x = pose.pose.position.x
            waypoint.y = pose.pose.position.y
            waypoint.theta = math.atan2(
                2.0
                * (
                    pose.pose.orientation.w * pose.pose.orientation.z
                    + pose.pose.orientation.x * pose.pose.orientation.y
                ),
                1.0
                - 2.0
                * (
                    pose.pose.orientation.y * pose.pose.orientation.y
                    + pose.pose.orientation.z * pose.pose.orientation.z
                ),
            )
            trajectory.waypoints.append(waypoint)
        self.publisher.publish(trajectory)
        blocked = self.blocked_waypoints(path)
        validation = "waiting for costmap validation" if blocked is None else f"blocked_points={blocked}"
        self.get_logger().info(
            f"Received A* path with {len(trajectory.waypoints)} waypoints; {validation}"
        )


def main():
    rclpy.init()
    node = AStarPathVisualizer()
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