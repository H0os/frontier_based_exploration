import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
import os
import time
from tf_transformations import euler_from_quaternion, quaternion_matrix
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer, LookupException

class OccupancyGridSubscriber(Node):
    def __init__(self):
        super().__init__('training_data_collector')

        # Subscription to 'map' topic
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.callback,
            10
        )

        # Parameter for saving path
        self.declare_parameter('path', '/workspace/')
        self.path = self.get_parameter('path').get_parameter_value().string_value

        # Create timestamped directory for saved images
        timestamp = time.strftime("%d_%m_%Y_%H_%M_%S", time.localtime())
        self.path = os.path.join(self.path, timestamp)
        os.makedirs(self.path, exist_ok=True)

        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Variables for motion tracking
        self.last_pose = None
        self.saved_image_count = 0

        # Frames
        self.odom_frame = "odom"
        self.base_frame = "base_footprint"

    def callback(self, msg):
        # Check if the robot has moved enough
        if not self.get_dist_travelled(2.0):  # 2 meters threshold
            return

        # Process occupancy grid data
        processed = self.preprocess_map(np.array(msg.data), msg.info.width, msg.info.height)

        # Convert to OpenCV image
        occupancy_grid = np.array(processed).reshape((msg.info.height, msg.info.width)).astype(np.int8)
        grayscale_image = np.full_like(occupancy_grid, 255, dtype=np.uint8)  # Default white
        grayscale_image[occupancy_grid == -1] = 128  # Unknown cells
        grayscale_image[occupancy_grid >= 1] = 0     # Occupied cells

        # Save the image
        filename = f"{self.saved_image_count}_{msg.info.height}_{msg.info.width}_{msg.info.resolution}.png"
        filepath = os.path.join(self.path, filename)
        cv2.imwrite(filepath, grayscale_image)

        self.saved_image_count += 1
        self.get_logger().info(f"Saved image: {filepath}")

    def preprocess_map(self, data, width, height):
        """
        Preprocesses the occupancy grid data.
        For simplicity, this function currently just reshapes the data.
        """
        return data.reshape((height, width))

    def get_dist_travelled(self, thresh):
        """
        Checks if the robot has moved a certain distance since the last recorded pose.

        :param thresh: Distance threshold in meters
        :return: True if the robot has moved beyond the threshold
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=3.0)
            )
        except LookupException as e:
            self.get_logger().error(f"Transform lookup failed: {e}")
            return False

        # Extract translation and rotation
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        current_pose = np.array([
            translation.x,
            translation.y,
            translation.z
        ])

        if self.last_pose is None:
            self.last_pose = current_pose
            return True

        # Calculate movement distance
        distance = np.linalg.norm(current_pose - self.last_pose)
        self.get_logger().info(f"Distance traveled: {distance:.2f} meters")

        if distance < thresh:
            return False

        # Update the last pose
        self.last_pose = current_pose
        return True


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
