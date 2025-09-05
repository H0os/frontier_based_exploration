#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

import numpy as np
import time
from math import sqrt
from collections import deque

# For plotting
import matplotlib
matplotlib.use('Agg')  # Use a non-interactive backend so we can save figures without a display
import matplotlib.pyplot as plt


class WavefrontFrontierCentroidPublisher(Node):
    def __init__(self):
        super().__init__('wavefront_frontier_centroid_publisher')

        # --- Parameters / Constants ---
        self.UNKNOWN = -1
        self.FREE = 0
        self.OCCUPIED = 100

        # Map storage
        self.occupancy_grid = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0

        # Robot pose (optional)
        self.robot_odom = None
        self.robot_x = 0.0
        self.robot_y = 0.0

        # Subscriptions
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publisher for frontier centroids
        self.frontier_pub = self.create_publisher(
            Float32MultiArray,
            'frontier_centroids',
            10
        )

        # Create a periodic timer (e.g., every 5 seconds)
        self.timer_period = 5.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # --- Performance tracking ---
        # We'll store a list of (pass_index, time_taken, cells_evaluated)
        self.evaluation_data = []
        self.pass_index = 0

        self.get_logger().info("WavefrontFrontierCentroidPublisher initialized. Waiting for map & odom...")

    # --------------------------------------------------------------
    # Callbacks
    # --------------------------------------------------------------
    def map_callback(self, msg: OccupancyGrid):
        """Store the map data in local variables."""
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

        grid_data = np.array(msg.data, dtype=np.int8).reshape((self.map_height, self.map_width))
        self.occupancy_grid = grid_data

    def odom_callback(self, msg: Odometry):
        """Store the robot odom in local variables (optional)."""
        self.robot_odom = msg
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def timer_callback(self):
        """
        Periodically find & publish frontier centroids, 
        and measure performance (time + cells evaluated).
        """
        if self.occupancy_grid is None:
            self.get_logger().warn("No map received yet, skipping frontier detection.")
            return

        # Start timing
        start_time = time.time()

        # We'll count cells_evaluated as the # of cells we examine in find_frontier_cells.
        # That function scans the entire map (width * height).
        # BFS expansions for clustering add overhead, but let's keep it simple for demonstration.
        # We'll store that count in a local var and pass it around.
        frontier_cells, cells_evaluated = self.find_frontier_cells()
        duration = time.time() - start_time
        self.log_performance_metrics(duration, cells_evaluated)
        self.pass_index += 1
        if not frontier_cells:
            # If no frontiers, publish empty array
            self.publish_frontier_centroids([])
            
           
            return

        # 2) Cluster frontier cells
        clusters = self.cluster_frontiers(frontier_cells)
        # 3) Compute centroids & sizes in world coords
        centroids_with_sizes = []
        for cluster in clusters:
            cx, cy = self.compute_cluster_centroid(cluster)
            size = len(cluster)
            wx, wy = self.map_to_world(cx, cy)
            centroids_with_sizes.append((wx, wy, size))

        # Publish
        self.publish_frontier_centroids(centroids_with_sizes)

        # Stop timing
        duration = time.time() - start_time

        # Log and store the performance metrics
        self.log_performance_metrics(duration, cells_evaluated)

        # Increment pass index
        self.pass_index += 1

        # (Optional) Generate a live performance plot after each pass
        self.update_and_save_plot()

    # --------------------------------------------------------------
    # Frontier Detection
    # --------------------------------------------------------------
    def find_frontier_cells(self):
        """
        Returns a tuple: (frontier_cells, cells_evaluated).
        A cell is a frontier if:
         - It is FREE (0),
         - It has at least one neighbor that is UNKNOWN (-1).
        """
        frontier_cells = []
        cells_evaluated = 0

        for my in range(self.map_height):
            for mx in range(self.map_width):
                cells_evaluated += 1  # We examined this cell
                if self.is_free(mx, my):
                    # Check neighbors for UNKNOWN
                    neighbors = self.get_4_neighbors(mx, my)
                    if any(self.is_unknown(nx, ny) for nx, ny in neighbors):
                        frontier_cells.append((mx, my))
        return frontier_cells, cells_evaluated

    # --------------------------------------------------------------
    # Clustering
    # --------------------------------------------------------------
    def cluster_frontiers(self, frontier_cells):
        """
        Group frontier cells into contiguous clusters (8-connected BFS).
        Return a list of clusters, each cluster is a list of (mx, my).
        """
        if not frontier_cells:
            return []

        frontier_set = set(frontier_cells)
        visited = set()
        clusters = []

        for cell in frontier_cells:
            if cell in visited:
                continue
            cluster = self.bfs_cluster(cell, frontier_set, visited)
            clusters.append(cluster)

        return clusters

    def bfs_cluster(self, start_cell, frontier_set, visited):
        """Collect all frontier cells connected to 'start_cell' via 8-direction BFS."""
        cluster = []
        queue = deque([start_cell])
        visited.add(start_cell)
        cluster.append(start_cell)

        while queue:
            cx, cy = queue.popleft()
            for nx, ny in self.get_8_neighbors(cx, cy):
                if (nx, ny) in frontier_set and (nx, ny) not in visited:
                    visited.add((nx, ny))
                    queue.append((nx, ny))
                    cluster.append((nx, ny))

        return cluster

    def compute_cluster_centroid(self, cluster):
        """
        Compute centroid of a cluster in MAP coordinates (floats).
        Returns (mx, my).
        """
        sx = 0.0
        sy = 0.0
        count = len(cluster)
        for (mx, my) in cluster:
            sx += mx
            sy += my
        return (sx / count, sy / count)

    # --------------------------------------------------------------
    # Publishing
    # --------------------------------------------------------------
    def publish_frontier_centroids(self, centroids_with_sizes):
        """
        Publishes frontier centroids in the format: 
        [x, y, size, x, y, size, ...] to a Float32MultiArray.
        """
        msg = Float32MultiArray()
        flat_data = []
        for (wx, wy, size) in centroids_with_sizes:
            flat_data.append(wx)
            flat_data.append(wy)
            flat_data.append(float(size))
        msg.data = flat_data
        self.frontier_pub.publish(msg)
        self.get_logger().info(f"Published {len(centroids_with_sizes)} frontier centroids.")

    # --------------------------------------------------------------
    # Utility
    # --------------------------------------------------------------
    def is_free(self, mx, my):
        """Check if map cell is FREE."""
        if not self.in_bounds(mx, my):
            return False
        return self.occupancy_grid[my, mx] == self.FREE

    def is_unknown(self, mx, my):
        """Check if map cell is UNKNOWN."""
        if not self.in_bounds(mx, my):
            return False
        return self.occupancy_grid[my, mx] == self.UNKNOWN

    def in_bounds(self, mx, my):
        """Check if map cell indices are within map bounds."""
        return (0 <= mx < self.map_width) and (0 <= my < self.map_height)

    def get_4_neighbors(self, mx, my):
        """Return 4-connected neighbors for (mx, my)."""
        return [
            (mx - 1, my),
            (mx + 1, my),
            (mx, my - 1),
            (mx, my + 1),
        ]

    def get_8_neighbors(self, mx, my):
        """Return 8-connected neighbors (including diagonals)."""
        return [
            (mx - 1, my),
            (mx + 1, my),
            (mx, my - 1),
            (mx, my + 1),
            (mx - 1, my - 1),
            (mx - 1, my + 1),
            (mx + 1, my - 1),
            (mx + 1, my + 1),
        ]

    def map_to_world(self, mx, my):
        """
        Convert map coordinates (mx, my) to world coordinates (wx, wy).
        We'll place the centroid in the center of the cell by adding 0.5.
        """
        wx = (mx + 0.5) * self.map_resolution + self.map_origin_x
        wy = (my + 0.5) * self.map_resolution + self.map_origin_y
        return (wx, wy)

    # --------------------------------------------------------------
    # Performance Tracking & Plotting
    # --------------------------------------------------------------
    def log_performance_metrics(self, duration, cells_evaluated):
        """
        Store & log performance metrics for the current pass.
        """
        self.evaluation_data.append((self.pass_index, duration, cells_evaluated))
        self.get_logger().info(
            f"Pass {self.pass_index} took {duration:.3f}s, "
            f"evaluated {cells_evaluated} cells."
        )
        with open("wavefront_performance_metric", "a") as f:
            f.write(f"{self.pass_index},{duration:.3f},{cells_evaluated}\n")

    def update_and_save_plot(self):
        """
        Creates or updates a matplotlib figure of pass_index vs. time 
        and pass_index vs. cells evaluated, then saves to disk.
        """
        if not self.evaluation_data:
            return

        # Separate data
        pass_indices = [d[0] for d in self.evaluation_data]
        durations = [d[1] for d in self.evaluation_data]
        cells_eval = [d[2] for d in self.evaluation_data]

        plt.figure(figsize=(8, 6))
        plt.suptitle("Frontier Detection Performance Over Time")

        # Subplot 1: Pass index vs. Duration
        plt.subplot(2, 1, 1)
        plt.plot(pass_indices, durations, marker='o', color='blue')
        plt.xlabel("Pass Index")
        plt.ylabel("Time (s)")
        plt.title("Computation Time per Pass")

        # Subplot 2: Pass index vs. Cells Evaluated
        plt.subplot(2, 1, 2)
        plt.plot(pass_indices, cells_eval, marker='o', color='green')
        plt.xlabel("Pass Index")
        plt.ylabel("Cells Evaluated")
        plt.title("Cells Evaluated per Pass")

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])  # to accommodate suptitle
        plt.savefig("evaluation_metrics.png")
        plt.close()

        self.get_logger().info("Updated performance plot saved to 'evaluation_metrics.png'.")


def main(args=None):
    rclpy.init(args=args)
    node = WavefrontFrontierCentroidPublisher()

    # try:
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     node.destroy_timer(node.timer)
    #     node.destroy_node()
    #     rclpy.shutdown()

    try:
        while rclpy.ok():
            # Spin once handles any pending callbacks with a small timeout
            rclpy.spin_once(node, timeout_sec=0.1)

            # Sleep or yield in between. Adjust if needed.
            time.sleep(2.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
