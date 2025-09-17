#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry 
from std_msgs.msg import Float32MultiArray, Bool
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
from tf_transformations import euler_from_quaternion, quaternion_matrix, quaternion_from_euler
import numpy as np
from collections import deque
import math, random
import time

class FrontierExploration(Node):
    def __init__(self):
        super().__init__('exploration')

        # Parameters
        self.centroids = []
        self.map_resolution = None
        self.map_origin = None
        self.map_data = None
        self.current_odom = None
        self.attempted_frontiers = set()
        self.current_goal = None
        self.start_time = time.time()
        self.explored_area = 0.0
        self.previous_map = None

        # Declare ROS2 Parameters for weights
        self.declare_parameter('weight_w1', 100)
        self.declare_parameter('weight_w2', 5.92627)
        self.declare_parameter('weight_w3', 0.7)
        self.w1 = self.get_parameter('weight_w1').value
        self.w2 = self.get_parameter('weight_w2').value
        self.w3 = self.get_parameter('weight_w3').value


        # Topics
        self.map_topic = '/map'
        self.odom_topic = '/odom'

        # Subscribers
        self.subscription = self.create_subscription(Float32MultiArray, 'frontier_centroids', self.frontier_callback, 10)
    
        self.subscription

     #   self.goal_state_pub = self.create_publisher(Bool, 'goal_state', 10)
        
        self.map_subscriber = self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, 10)
        
        self.odom_subscriber = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        self.metric_publisher = self.create_publisher(Float32MultiArray, 'exploration_metrics', 10)
        
        self.timer_metric = self.create_timer(1.0, self.publish_performance_metric)
                
        self.goal_status = None  # To track the current goal's status
        
        self.goal_status_subscriber = self.create_subscription(GoalStatus,'/navigate_to_pose/status', self.goal_status_callback, 10)
        # Action Client for Navigation
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Timer for periodic exploration
        # msg = Bool()
        # msg.data = False
        # self.goal_state_pub.publish(msg)

        self.timer = self.create_timer(0.25, self.explore_callback)

        self.get_logger().info("Frontier Exploration Node Initialized!")

    def goal_status_callback(self, msg):
        """Callback to track the status of the current navigation goal."""
        if msg.status_list:
            # Get the last status in the status list
            latest_status = msg.status_list[-1].status
            self.goal_status = latest_status

            status_dict = {
                GoalStatus.STATUS_UNKNOWN: "Unknown",
                GoalStatus.STATUS_ACCEPTED: "Accepted",
                GoalStatus.STATUS_EXECUTING: "Executing",
                GoalStatus.STATUS_SUCCEEDED: "Succeeded",
                GoalStatus.STATUS_CANCELED: "Canceled",
                GoalStatus.STATUS_ABORTED: "Aborted",
            }
            self.get_logger().info(f"Current goal status: {status_dict.get(latest_status, 'Unknown')}")
        else:
            self.goal_status = None  # No active goal
            self.get_logger().info("No active navigation goal.")



    def map_callback(self, msg):
        """Callback to update the map and track explored area."""
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin
        new_map_data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))

        if self.previous_map is not None:
            # Ensure dimensions match by padding or cropping the smaller map
            if new_map_data.shape != self.previous_map.shape:
                self.previous_map = self.adjust_map_size(self.previous_map, new_map_data.shape)

            # Calculate newly discovered area
            new_explored = np.logical_and(new_map_data == 0, self.previous_map != 0)
            newly_discovered_area = np.sum(new_explored) * (self.map_resolution ** 2)
            self.explored_area += newly_discovered_area

        self.previous_map = new_map_data.copy()

    def adjust_map_size(self, map_to_adjust, target_shape):
        """
        Adjust the size of a map by padding or cropping to match the target shape.

        :param map_to_adjust: The map to adjust (numpy array).
        :param target_shape: The target shape (tuple: height, width).
        :return: The adjusted map (numpy array).
        """
        current_shape = map_to_adjust.shape
        if current_shape == target_shape:
            return map_to_adjust  # No adjustment needed

        # Padding or cropping
        adjusted_map = np.zeros(target_shape, dtype=map_to_adjust.dtype)

        # Determine cropping/padding ranges
        min_height = min(current_shape[0], target_shape[0])
        min_width = min(current_shape[1], target_shape[1])

        adjusted_map[:min_height, :min_width] = map_to_adjust[:min_height, :min_width]

        return adjusted_map


    def odom_callback(self, msg):
        """Callback to update current odometry data."""
        self.current_odom = msg



    def explore_callback(self):
        """Callback to handle exploration and goal management."""
        # Check if there is an active goal
        if self.goal_status in [GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_ACCEPTED]:
          #  self.get_logger().info("Waiting for the current goal to be reached.")
            # robot_pos = self.get_robot_position()
            # if robot_pos:
            #     rx, ry, _ = robot_pos
            #     gx, gy = self.current_goal[0], self.current_goal[1]
            #     dist = math.sqrt((gx - rx)**2 + (gy - ry)**2)

            #     # If distance is below some threshold, say 0.5
            #     if dist < 0.5:
            #         self.get_logger().info(f"Within 0.5 m of the goal ({gx:.2f}, {gy:.2f}). Publishing False.")
                    
                    # msg = Bool()
                    # msg.data = False
                    # self.goal_state_pub.publish(msg)
            return

        # Handle completed or aborted goals
        if self.goal_status in [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED]:
            self.get_logger().info("Previous goal completed or canceled. Selecting a new goal.")
            self.goal_status = None  # Reset the status for the next goal

        # If no frontiers are available, exploration is complete
        frontiers = self.centroids
        if not frontiers:
            self.get_logger().info("No frontiers found. Exploration is complete.")
            self.navigate_to_random_spot()
            return

        # Choose the closest frontier
        closest_frontier = self.find_closest_frontier(frontiers)
        if closest_frontier:
            self.get_logger().info(f"Navigating to new frontier: {closest_frontier}")
            self.navigate_to_goal(closest_frontier)
        else:
            self.get_logger().info("No valid frontier could be selected.")
            self.navigate_to_random_spot(radius=3)

    def find_closest_frontier(self, frontiers):
        """Find the closest frontier to the robot."""    

        if self.current_odom is None:
            self.get_logger().warn("Odometry data not available.")
            return None

        # Get the robot's current position
        robot_position = self.get_robot_position()
        if robot_position is None:
            self.get_logger().warn("Robot position could not be determined.")
            return None
        
        robot_x, robot_y, robot_orientation = robot_position
        min_distance = float('inf')
        min_cost = float('inf')
        closest_frontier = None

        for frontier in frontiers:
            distance = math.sqrt((frontier[0] - robot_x)**2 + (frontier[1] - robot_y)**2)
            angle_to_frontier = math.atan2(frontier[1] - robot_y, frontier[0] - robot_x)
            
            angular_difference = abs(angle_to_frontier - robot_orientation)
            angular_difference = min(angular_difference, 2 * math.pi - angular_difference)

            cost = self.w1 * distance + self.w2 * angular_difference - self.w3 * frontier[2]
            if (cost < min_cost) and (distance > 0.3):
                min_cost = cost
                closest_frontier = frontier
        
        return closest_frontier


    def get_robot_position(self):
        """Get the robot's current position in world coordinates."""
        if self.current_odom is None:
            self.get_logger().warn("Odometry data not available.")
            return None

        try:
            # Extract position and orientation from odometry
            position = self.current_odom.pose.pose.position
            orientation = self.current_odom.pose.pose.orientation
            roll, pitch, yaw = euler_from_quaternion(
                [orientation.x, orientation.y, orientation.z, orientation.w]
            )
            return (position.x, position.y, yaw)
        except Exception as e:
            self.get_logger().error(f"Error retrieving robot position: {e}")
            return None


    def grid_to_world(self, x, y):
        """Convert grid coordinates to world coordinates."""
        if self.map_origin is None or self.map_resolution is None:
            self.get_logger().warn("Map origin or resolution is not set.")
            return None, None

        # Convert grid to relative coordinates
        relative_x = x * self.map_resolution
        relative_y = y * self.map_resolution

        # Apply map origin transformation
        origin = self.map_origin.position
        orientation = self.map_origin.orientation
        rotation_matrix = quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])

        # Apply rotation
        rotated = np.dot(rotation_matrix[:2, :2], np.array([relative_x, relative_y]))

        # Translate to world coordinates
        world_x = origin.x + rotated[0]
        world_y = origin.y + rotated[1]
        return world_x, world_y


    def world_to_grid(self, world_x, world_y):
        """Convert world coordinates to grid coordinates."""
        grid_x = int((world_x - self.map_origin.position.x) / self.map_resolution)
        grid_y = int((world_y - self.map_origin.position.y) / self.map_resolution)
        return grid_x, grid_y


    def navigate_to_random_spot(self, radius=0.5):
        """Navigate to a random spot within a small radius around the robot."""
        if not self.nav_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigation action server not available!")
            return
        
        robot_position = self.get_robot_position()
        if robot_position is None:
            self.get_logger().error("Failed to retrieve robot position.")
            return
        
        x, y, yaw = robot_position
        random_angle = random.uniform(0, 2 * 3.14159)
        random_distance = random.uniform(1, radius)
        
        new_x = x + random_distance * math.cos(random_angle)
        new_y = y + random_distance * math.sin(random_angle)

        self.navigate_to_goal(goal=(new_x,new_y,1))


    def navigate_to_goal(self, goal):
        """Send a navigation goal to the Nav2 stack."""
        if not self.nav_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigation action server not available!")
            return

        # Set the goal as active

        # msg = Bool()
        # msg.data = True
        # self.goal_state_pub.publish(msg)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal[0]
        goal_msg.pose.pose.position.y = goal[1]
        goal_msg.pose.pose.orientation.w = 1.0  # No rotation
        self.attempted_frontiers.add(goal)
        self.get_logger().info(f"Sending navigation goal: {goal}")
        send_goal_future = self.nav_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)



    def goal_response_callback(self, future):
        """Handle response from Nav2 when a goal is sent."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            self.goal_status = GoalStatus.STATUS_ABORTED#

            # msg = Bool()
            # msg.data = False
            # self.goal_state_pub.publish(msg)

            return

        self.get_logger().info("Goal accepted. Monitoring progress...")
        self.goal_status = GoalStatus.STATUS_EXECUTING  # Update the status when accepted
        goal_handle.get_result_async().add_done_callback(
            lambda future: self.result_callback(future)
        )

    def result_callback(self, future):
        """Handle the result of the navigation goal."""
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal reached successfully.")
            self.goal_status = GoalStatus.STATUS_SUCCEEDED
        elif result.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info("Goal was aborted.")
            self.goal_status = GoalStatus.STATUS_ABORTED
        elif result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Goal was canceled.")
            self.goal_status = GoalStatus.STATUS_CANCELED

        #msg = Bool()
        #msg.data = False
        #self.goal_state_pub.publish(msg)


    def frontier_callback(self, msg):
        """
        Callback function to process received frontier centroids.

        :param msg: The received Float32MultiArray message
        """
        data = msg.data

        # Parse the received data into centroids and sizes
        self.centroids = []
        for i in range(0, len(data), 3):
            x = data[i]
            y = data[i + 1]
            
            size = data[i + 2]
            if (x,y,size) not in self.attempted_frontiers:
                self.centroids.append((x, y, size))

        #self.get_logger().info(f"Received {len(self.centroids)} frontier centroids")
        #for centroid in self.centroids:
            #self.get_logger().info(f"Centroid: x={centroid[0]:.2f}, y={centroid[1]:.2f}, size={centroid[2]:.2f}")

    def compute_performance_metric(self):
        """Calculate the performance metric: area discovered / time taken."""
        elapsed_time = time.time() - self.start_time
        return self.explored_area / elapsed_time if elapsed_time > 0 else 0.0

    def publish_performance_metric(self):
        """Publish the performance metric."""
        metric = self.compute_performance_metric()
        msg = Float32MultiArray()
        msg.data = [metric, self.explored_area, time.time() - self.start_time]
        self.metric_publisher.publish(msg)
        


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExploration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

