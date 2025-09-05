#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray, Bool
import numpy as np
import argparse
import torch
from PIL import Image
from torchvision import transforms
from ultralytics import YOLO
import cv2
from tf_transformations import euler_from_quaternion, quaternion_matrix, quaternion_from_euler
from frontier_exploration.tree_node import build_tree, search, TreeNode, visualize_tree_on_map, remove_searched_nodes_bfs, get_grid_size

# -- ADDED/CHANGED: For plotting performance metrics
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend (for saving .png files)
import matplotlib.pyplot as plt

class MapUtils(Node):
    def __init__(self, find_frontiers_flag, learned_find_frontiers_flag):
        super().__init__('map_utils')

        self.find_frontiers_flag = find_frontiers_flag
        self.learned_find_frontiers_flag = learned_find_frontiers_flag
        self.root = None
        self.node_set = set()
        self.current_odom = None
        self.odom_topic = '/odom'
        self.map_resolution = None
        self.map_origin = None
        self.exploring = None

        self.current_map_info = None  # Will hold the most recent map's metadata
        self.last_map_info = None     # Will hold the previous map's metadata
        self.offset_x = 0
        self.offset_y = 0

        # If you're using a trained model
        if self.learned_find_frontiers_flag:
            self.model = YOLO("/home/aymon/frontier_exploration_ws/frontier_exploration/"
                              "frontier_exploration/best.pt")
            self.model.cpu()

        # Subscribe to the 'map' topic which publishes OccupancyGrid messages
        self.subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        # Publisher for frontier centroids with sizes
        if self.find_frontiers_flag or self.learned_find_frontiers_flag:
            self.frontier_pub = self.create_publisher(Float32MultiArray, 'frontier_centroids', 10)

        # -- ADDED/CHANGED: Performance tracking data structures
        self.evaluation_data = []  # Will store tuples (pass_index, time_seconds, cells_evaluated)
        self.pass_index = 0
        self.output_txt_file = "map_utils_performance.txt" 

        self.get_logger().info("MapUtils node initialized.")

    def goal_state_callback(self, msg):
        self.exploring = msg.data
        if msg.data:
            self.get_logger().info("Exploration node says: Robot has an active goal (True).")
        else:
            self.get_logger().info("Exploration node says: Robot has NO active goal (False).")

    def odom_callback(self, msg):
        """Callback to update current odometry data."""
        self.current_odom = msg

    def map_callback(self, msg):
        """
        Callback function that processes the OccupancyGrid message.
        This runs repeatedly as new map updates come in (or on a set schedule).
        """
        if self.current_map_info is not None:
            # Save the current map info as 'last_map_info' before updating
            self.last_map_info = self.current_map_info

        # Update current map info
        self.current_map_info = msg.info

        # If we already have a 'last_map_info', compute offset
        if self.last_map_info is not None:
            self.offset_x, self.offset_y = self.compute_map_offset(self.last_map_info, self.current_map_info)
            self.get_logger().info(
                f"Map origin offset (in grid coords): Δx={self.offset_x}, Δy={self.offset_y}"
            )


        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        self.map_resolution = resolution
        origin = msg.info.origin.position
        self.map_origin = msg.info.origin

        self.get_logger().info(f"Map received: {width}x{height} at {resolution} m/pixel")
        self.get_logger().info(f"Map origin: x={origin.x}, y={origin.y}, z={origin.z}")

        grid_data = np.array(msg.data).reshape((height, width))

        # Example: count occupied, free, unknown
        occupied = np.sum(grid_data == 100)
        free = np.sum(grid_data == 0)
        unknown = np.sum(grid_data == -1)
        self.get_logger().info(f"Occupied: {occupied}, Free: {free}, Unknown: {unknown}")

        # If user requested frontier detection
        if self.find_frontiers_flag:
            robot_position = self.get_robot_position()
            if not robot_position:
                return

            # -- ADDED/CHANGED: We'll measure performance around find_frontiers()
            start_time = time.time()
            frontiers, grids_searched = self.find_frontiers(grid_data, resolution, origin)
            duration = time.time() - start_time

            centroids = self.cluster_and_find_centroids(frontiers)

            #remove_searched_nodes_bfs(self.root)
    
            # Visualize
            visualize_tree_on_map(grid_data, self.root, "my_tree_overlay.png")


            # For "cells evaluated," we assume the entire grid is relevant:
            cells_evaluated = grids_searched * (get_grid_size() ** 2)

            # Store performance data
            self.log_performance_metrics(duration, cells_evaluated)

            # Publish the frontier centroids
            self.publish_frontier_centroids(centroids)

            # Optionally, generate/update a plot each time
            self.update_and_save_plot()

        # If user requested learned frontier detection
        if self.learned_find_frontiers_flag:
            start_time = time.time()
            centroids = self.learned_find_frontiers(grid_data, resolution, origin)
            duration = time.time() - start_time

            # For "cells evaluated," you might adapt based on how your ML model processes data
            cells_evaluated = grid_data.size

            self.log_performance_metrics(duration, cells_evaluated)

            self.publish_frontier_centroids(centroids)
            self.update_and_save_plot()

    def get_robot_position(self):
        """Get the robot's current position in world coordinates."""
        if self.current_odom is None:
            self.get_logger().warn("Odometry data not available.")
            return None

        try:
            position = self.current_odom.pose.pose.position
            orientation = self.current_odom.pose.pose.orientation
            roll, pitch, yaw = euler_from_quaternion(
                [orientation.x, orientation.y, orientation.z, orientation.w]
            )
            return (position.x, position.y, yaw)
        except Exception as e:
            self.get_logger().error(f"Error retrieving robot position: {e}")
            return None

    def world_to_grid(self, world_x, world_y, origin, resolution):
        """Convert world coordinates to grid coordinates."""
        grid_x = int((world_x - origin.x) / resolution)
        grid_y = int((world_y - origin.y) / resolution)
        return grid_x, grid_y

    def grid_to_world(self, x, y):
        """Convert grid coordinates to world coordinates."""
        if self.map_origin is None or self.map_resolution is None:
            self.get_logger().warn("Map origin or resolution is not set.")
            return None, None

        relative_x = x * self.map_resolution
        relative_y = y * self.map_resolution

        origin = self.map_origin.position
        orientation = self.map_origin.orientation
        rotation_matrix = quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])

        rotated = np.dot(rotation_matrix[:2, :2], np.array([relative_x, relative_y]))

        world_x = origin.x + rotated[0]
        world_y = origin.y + rotated[1]
        return world_x, world_y

    def find_frontiers(self, grid_data, resolution, origin):
        """
        Finds all frontier centroids using your QuadTree-based approach.
        We measure performance outside this function (in map_callback).
        """
        
        if not self.root:
            robot_position = self.get_robot_position()
            if not robot_position:
                return []
            world_robot_x, world_robot_y, _ = robot_position
            grid_robot_x, grid_robot_y = self.world_to_grid(world_robot_x, world_robot_y, origin, resolution)
            self.root = TreeNode(grid_robot_x, grid_robot_y)
            self.get_logger().warn(f"Root node at: {grid_robot_x}, {grid_robot_y}")

        # Build tree & search for frontiers
        self.node_set, unsearched_nodes = build_tree(grid_data, self.root, self.node_set, self.offset_x, self.offset_y)
        frontiers, grids_searched = search(grid_data, unsearched_nodes, self.map_resolution, self.map_origin)
      
        # Then cluster & find centroids
        
        return frontiers, grids_searched

    def cluster_and_find_centroids(self, frontiers):
        """
        Clusters frontier points and computes centroids with sizes.
        """
        from sklearn.cluster import DBSCAN
        import numpy as np

        if not frontiers:
            return []

        frontier_array = np.array(frontiers)
        clustering = DBSCAN(eps=0.5, min_samples=5).fit(frontier_array)

        centroids_with_sizes = []
        for cluster_label in set(clustering.labels_):
            if cluster_label == -1:  # Noise
                continue

            cluster_points = frontier_array[clustering.labels_ == cluster_label]
            centroid = np.mean(cluster_points, axis=0)
            size = len(cluster_points)
            centroids_with_sizes.append((centroid[0], centroid[1], size))

        return centroids_with_sizes

    def publish_frontier_centroids(self, centroids_with_sizes):
        """
        Publishes frontier centroids (x, y, size) to a topic as a single Float32MultiArray.
        """
        message = Float32MultiArray()
        flat_data = []
        for centroid in centroids_with_sizes:
            flat_data.extend([centroid[0], centroid[1], centroid[2]])
        message.data = flat_data
        self.frontier_pub.publish(message)
        self.get_logger().info(f"Published {len(centroids_with_sizes)} frontier centroids as a single message.")

    def learned_find_frontiers(self, grid_data, resolution, origin):
        """
        Finds frontiers using a YOLO-based model, etc.
        We measure performance outside this function (in map_callback).
        """
        img = np.ones_like(np.array(grid_data), dtype=np.uint8)*255
        img[np.array(grid_data) == -1] = 128
        img[np.array(grid_data) >= 1] = 0

        height, width = grid_data.shape
        img = np.reshape(img, (height, width))
        img0 = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

        results = self.model(img0)

        centroids_with_sizes = []
        for result in results:
            boxes = result.boxes  # bounding box outputs
            data = boxes.data
            x_min, y_min, x_max, y_max = data[:, 0], data[:, 1], data[:, 2], data[:, 3]

            x_center = (x_min + x_max) / 2
            y_center = (y_min + y_max) / 2

            rect_width = x_max - x_min
            rect_height = y_max - y_min
            area = rect_width * rect_height

            for i in range(len(data)):
                world_x = origin.x + (x_center[i] * resolution)
                world_y = origin.y + (y_center[i] * resolution)
                centroids_with_sizes.append((world_x, world_y, area[i]))
            
            result.save(filename="result.jpg")

            # e.g. result.save(filename="result.jpg") to visualize bounding boxes if desired

        return centroids_with_sizes

    # --------------------------------------------------------------
    # Performance Tracking & Plotting
    # --------------------------------------------------------------
    def log_performance_metrics(self, duration, cells_evaluated):
        """Store & log performance metrics for the current pass."""
        self.evaluation_data.append((self.pass_index, duration, cells_evaluated))
        self.get_logger().info(
            f"[Performance] Pass {self.pass_index} took {duration:.3f}s, "
            f"evaluated {cells_evaluated} cells."
        )

        with open(self.output_txt_file, "a") as f:
            f.write(f"{self.pass_index},{duration:.3f},{cells_evaluated}\n")

        self.pass_index += 1

    def update_and_save_plot(self):
        """
        Creates or updates a matplotlib figure:
           - pass_index vs. time
           - pass_index vs. cells_evaluated
        Then saves to disk (e.g. 'performance_metrics.png').
        """
        if not self.evaluation_data:
            return

        pass_indices = [d[0] for d in self.evaluation_data]
        durations = [d[1] for d in self.evaluation_data]
        cells_eval = [d[2] for d in self.evaluation_data]

        plt.figure(figsize=(8, 6))
        plt.suptitle("Frontier Detection Performance Over Time")

        # Subplot 1: Time vs. Pass
        plt.subplot(2, 1, 1)
        plt.plot(pass_indices, durations, marker='o', color='blue')
        plt.xlabel("Pass Index")
        plt.ylabel("Time (s)")
        plt.title("Computation Time per Pass")

        # Subplot 2: Cells Evaluated vs. Pass
        plt.subplot(2, 1, 2)
        plt.plot(pass_indices, cells_eval, marker='o', color='green')
        plt.xlabel("Pass Index")
        plt.ylabel("Cells Evaluated")
        plt.title("Cells Evaluated per Pass")

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.savefig("graph_performance_metrics.png")
        plt.close()

        self.get_logger().info("Updated performance plot saved to 'performance_metrics.png'.")

    def compute_map_offset(self, old_map_info, new_map_info):
        """
        Compute how many grid cells the map's origin has shifted 
        between 'old_map_info' and 'new_map_info'.
        
        Returns:
            (offset_x, offset_y): The shift in grid cell indices.
        """
        # Extract the old and new origins in world coordinates
        old_origin = old_map_info.origin.position
        new_origin = new_map_info.origin.position

        # Calculate the difference in meters
        dx_meters = new_origin.x - old_origin.x
        dy_meters = new_origin.y - old_origin.y

        # Convert meters to grid cell indices using the NEW map's resolution
        # (You may decide to use the old map's resolution if that better suits your logic.)
        cell_x_offset = int(round(dx_meters / new_map_info.resolution))
        cell_y_offset = int(round(dy_meters / new_map_info.resolution))

        return (cell_x_offset, cell_y_offset)
    
def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--find_frontiers', action='store_true', help="Flag to enable frontier detection")
    parser.add_argument('--learned_find_frontiers', action='store_true', help="Flag to enable frontier detection using model")

    args, unknown = parser.parse_known_args()
    rclpy.init(args=unknown)

    # Create the node
    map_utils = MapUtils(find_frontiers_flag=args.find_frontiers,
                         learned_find_frontiers_flag=args.learned_find_frontiers)

    # Spin the node so the callback is processed
    try:
        while rclpy.ok():
            # Spin once handles any pending callbacks with a small timeout
            rclpy.spin_once(map_utils, timeout_sec=0.1)

            # Sleep or yield in between. Adjust if needed.
            time.sleep(2.0)
    except KeyboardInterrupt:
        pass
    finally:
        map_utils.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
