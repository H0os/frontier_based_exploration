import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
import numpy as np
import argparse
import torch
from PIL import Image
from torchvision import transforms
from ultralytics import YOLO
import cv2

class MapUtils(Node):
    def __init__(self, find_frontiers_flag, learned_find_frontiers_flag):
        super().__init__('map_utils')
        self.find_frontiers_flag = find_frontiers_flag
        self.learned_find_frontiers_flag = learned_find_frontiers_flag
        
        if self.learned_find_frontiers_flag:
            # self.get_logger().info("Loading TorchScript model for learned frontier detection...")
            # self.model = torch.jit.load("/home/aymon/frontier_exploration_ws/frontier_exploration/frontier_exploration/best.torchscript")
            # self.model.eval()
            # self.get_logger().info("TorchScript model loaded successfully!")

            # # Define preprocessing for the model
            # self.transform = transforms.Compose([
            #     transforms.ToPILImage(),
            #     transforms.Resize((256, 256)),  # Adjust to model input size
            #     transforms.ToTensor(),
            #     transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
            # ])
            self.model = YOLO("/home/aymon/frontier_exploration_ws/frontier_exploration/frontier_exploration/best.pt")

        # Subscribe to the 'map' topic which publishes OccupancyGrid messages
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Publisher for frontier centroids with sizes
        if self.find_frontiers_flag:
            self.frontier_pub = self.create_publisher(Float32MultiArray, 'frontier_centroids', 10)

        if self.learned_find_frontiers_flag:
            self.frontier_pub = self.create_publisher(Float32MultiArray, 'frontier_centroids', 10)

    def map_callback(self, msg):
        """
        Callback function that processes the OccupancyGrid message.

        :param msg: The received OccupancyGrid message
        """
        # Get metadata from the OccupancyGrid
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin.position

        self.get_logger().info(f"Map received: {width}x{height} at {resolution} m/pixel")
        self.get_logger().info(f"Map origin: x={origin.x}, y={origin.y}, z={origin.z}")

        # Convert the OccupancyGrid data to a numpy array
        grid_data = np.array(msg.data).reshape((height, width))

        # Example: Count occupied, free, and unknown cells
        occupied = np.sum(grid_data == 100)
        free = np.sum(grid_data == 0)
        unknown = np.sum(grid_data == -1)

        self.get_logger().info(f"Occupied: {occupied}, Free: {free}, Unknown: {unknown}")

        # Additional processing
        #self.save_map_as_image(grid_data, resolution, origin)
        #self.find_largest_free_space(grid_data)

        if self.find_frontiers_flag:
            centroids = self.find_frontiers(grid_data, resolution, origin)
            self.publish_frontier_centroids(centroids)
        
        if self.learned_find_frontiers_flag:
            centroids = self.learned_find_frontiers(grid_data, resolution, origin)
            self.publish_frontier_centroids(centroids)

    def save_map_as_image(self, grid_data, resolution, origin):
        """
        Saves the map data as an image file for visualization.

        :param grid_data: 2D numpy array of the map data
        :param resolution: Resolution of the map (meters per pixel)
        :param origin: Origin of the map in world coordinates
        """
        import matplotlib.pyplot as plt

        # Create a colormap for visualization
        colormap = {
            -1: [0.5, 0.5, 0.5],  # Unknown cells (gray)
            0: [1, 1, 1],        # Free cells (white)
            100: [0, 0, 0]       # Occupied cells (black)
        }

        # Apply colormap
        image = np.zeros((grid_data.shape[0], grid_data.shape[1], 3))
        for value, color in colormap.items():
            image[grid_data == value] = color

        # Save the image
        plt.imshow(image, origin='lower')
        plt.title("Occupancy Grid Map")
        plt.xlabel("X (pixels)")
        plt.ylabel("Y (pixels)")
        plt.savefig("occupancy_grid_map.png")
        self.get_logger().info("Map saved as occupancy_grid_map.png")

    def find_largest_free_space(self, grid_data):
        """
        Identifies the largest contiguous free space in the map.

        :param grid_data: 2D numpy array of the map data
        """
        from scipy.ndimage import label

        # Identify free spaces (value == 0)
        free_space = (grid_data == 0).astype(int)

        # Label contiguous regions of free space
        labeled_array, num_features = label(free_space)

        # Find the largest region
        largest_region_size = 0
        largest_region_label = 0

        for region_label in range(1, num_features + 1):
            region_size = np.sum(labeled_array == region_label)
            if region_size > largest_region_size:
                largest_region_size = region_size
                largest_region_label = region_label

        self.get_logger().info(f"Largest free space: {largest_region_size} cells")

        # Optionally, visualize the largest region
        largest_region = (labeled_array == largest_region_label)
        self.visualize_region(largest_region)

    def visualize_region(self, region):
        """
        Visualizes a binary region as an image.

        :param region: 2D numpy array representing the region
        """
        import matplotlib.pyplot as plt

        plt.imshow(region, cmap="Greys", origin='lower')
        plt.title("Largest Free Space Region")
        plt.xlabel("X (pixels)")
        plt.ylabel("Y (pixels)")
        plt.savefig("largest_free_space.png")
        self.get_logger().info("Largest free space region saved as largest_free_space.png")

    def find_frontiers(self, grid_data, resolution, origin):
        """
        Finds all frontier centroids in the occupancy grid.

        :param grid_data: 2D numpy array of the map data
        :param resolution: Resolution of the map (meters per pixel)
        :param origin: Origin of the map in world coordinates
        :return: List of tuples containing centroids and their sizes
        """
        frontiers = []
        height, width = grid_data.shape

        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if grid_data[y, x] == -1:  # Unknown cell
                    neighbors = grid_data[y-1:y+2, x-1:x+2].flatten()
                    if 0 in neighbors:  # Check if adjacent to free space
                        world_x = origin.x + (x * resolution)
                        world_y = origin.y + (y * resolution)
                        frontiers.append((world_x, world_y))

        centroids_with_sizes = self.cluster_and_find_centroids(frontiers)
        return centroids_with_sizes

    def cluster_and_find_centroids(self, frontiers):
        """
        Clusters frontier points and computes centroids with sizes.

        :param frontiers: List of frontier points in world coordinates
        :return: List of tuples (centroid_x, centroid_y, size)
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
        Publishes frontier centroids and their sizes to a topic as a single message.

        :param centroids_with_sizes: List of tuples (centroid_x, centroid_y, size)
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
        Finds frontiers using the pre-trained model with corrected tensor reshaping.
        Saves the model's output as an image for visualization.
        # """
        # # Convert grid data to 3-channel image
        # grid_image = (grid_data + 1) * 127
        # grid_image = grid_image.astype(np.uint8)
        # grid_image = np.stack([grid_image] * 3, axis=-1)

        # # Preprocess and inference
        # input_tensor = self.transform(grid_image).unsqueeze(0)
        # with torch.no_grad():
        #     output = self.model(input_tensor)

        # # Fixed output dimensions based on model architecture
        # CHANNELS = 5
        # H = 32  # Model output height for 256x256 input
        # W = 42  # Model output width for 256x256 input

        # try:
        #     output_tensor = output[0].reshape(CHANNELS, H, W)
        # except RuntimeError as e:
        #     self.get_logger().error(f"Reshape failed: {str(e)}")
        #     return []

        # # Save the model's output as an image
        # self.save_model_output(output_tensor, "model_output.png")

        # # Process output tensor
        # frontier_map = output_tensor[0].numpy()
        # frontier_indices = np.argwhere(frontier_map > 0.5)

        # frontiers = []
        # for y, x in frontier_indices:
        #     # Scale coordinates to original map dimensions
        #     scaled_x = x * (256 / W) * (resolution * grid_data.shape[1] / 256)
        #     scaled_y = y * (256 / H) * (resolution * grid_data.shape[0] / 256)
            
        #     world_x = origin.x + scaled_x
        #     world_y = origin.y + scaled_y
        #     frontiers.append((world_x, world_y))

        # self.get_logger().info(f"Detected {len(frontiers)} raw frontier points")

        img = np.ones_like(np.array(grid_data), dtype=np.uint8)*255
        img[np.array(grid_data) == -1] = 128
        img[np.array(grid_data) >= 1] = 0

        # Reshape and flip to convert map -> pixel coordinates
        height , width = grid_data.shape
        img = np.reshape(img, (height, width))
        #img = cv2.flip(img, 0)

        img0 = img.copy()
        img0 = cv2.cvtColor(img0, cv2.COLOR_GRAY2RGB)

        results = self.model(img0)

        centroids_with_sizes = []        
        for result in results:
            boxes = result.boxes  # Boxes object for bounding box outputs
            data = boxes.data

            x_min, y_min, x_max, y_max = data[:, 0], data[:, 1], data[:, 2], data[:, 3]

            # Calculate center positions
            x_center = (x_min + x_max) / 2
            y_center = (y_min + y_max) / 2

            

            # Calculate areas
            width = x_max - x_min
            height = y_max - y_min
            area = width * height

            for i in range(len(data)):
                world_x = origin.x + (x_center[i] * resolution)
                world_y = origin.y + (y_center[i] * resolution)
                centroids_with_sizes.append((world_x, world_y, area[i]))

            result.save(filename="result.jpg")

        return centroids_with_sizes

    def save_model_output(self, output_tensor, filename):
        """
        Saves the model's output tensor as an image for visualization.

        :param output_tensor: The model's output tensor (CHANNELS, H, W)
        :param filename: The filename to save the image as
        """
        import matplotlib.pyplot as plt

        # Use the first channel for visualization (frontier map)
        frontier_map = output_tensor[0].numpy()

        # Normalize the output to [0, 1] for visualization
        frontier_map_normalized = (frontier_map - frontier_map.min()) / (frontier_map.max() - frontier_map.min())

        # Plot and save the image
        plt.figure(figsize=(10, 10))
        plt.imshow(frontier_map_normalized, cmap="viridis", origin="lower")
        plt.colorbar(label="Frontier Probability")
        plt.title("Model Output (Frontier Map)")
        plt.xlabel("X (pixels)")
        plt.ylabel("Y (pixels)")
        plt.savefig(filename)
        plt.close()

        self.get_logger().info(f"Model output saved as {filename}")



def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--find_frontiers', action='store_true', help="Flag to enable frontier detection")
    parser.add_argument('--learned_find_frontiers', action='store_true', help="Flag to enable frontier detection using model")

    args, unknown = parser.parse_known_args()

    rclpy.init(args=unknown)

    # Create the node
    map_utils = MapUtils(find_frontiers_flag=args.find_frontiers, learned_find_frontiers_flag=args.learned_find_frontiers)

    # Spin the node so the callback is processed
    rclpy.spin(map_utils)

    # Cleanup on shutdown
    map_utils.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

